"""Label-independent, deterministic sRGB low-light stress augmentation.

This approximates rendered RGB degradation, not calibrated RAW noise or lamps.
No segmentation labels are available to the illumination generator.
"""
import cv2
import numpy as np

VERSION = 'rgb_night_v2'
SEVERITIES = ('light', 'medium', 'strong')


def sample_parameters(rng, severity):
    ranges = {
        'light': ((1.25, 1.7), (.22, .38), (.45, .75), (.75, 1.2), (.003, .010)),
        'medium': ((1.6, 2.1), (.09, .22), (.6, .95), (.65, 1.25), (.005, .016)),
        'strong': ((1.9, 2.4), (.035, .12), (.7, 1.10), (.6, 1.3), (.008, .022)),
    }
    if severity not in ranges:
        raise ValueError('Unknown low-light severity')
    gamma, ambient, beam, contrast, read = ranges[severity]
    u = lambda a: float(rng.uniform(*a))
    return dict(version=VERSION, severity=severity, gamma=u(gamma), ambient=u(ambient),
                beam=u(beam), contrast=u(contrast), read_noise=u(read),
                shot_noise=u((.00002, .0002)), cx=u((.465, .535)), cy=u((.77, .88)),
                separation=u((.40, .48)), sx=u((.14, .18)), sy=u((.21, .28)),
                vertical_offset=u((-.025, .025)), asymmetry=u((-.18, .18)),
                shadow=u((.05, .3)), shadow_x=u((0., 1.)), shadow_y=u((.4, 1.)),
                glare=u((.05, .18)) if rng.random() < .15 else 0.,
                glare_x=u((.1, .9)), glare_y=u((.45, .95)),
                blur=int(rng.choice([0, 1, 2], p=[.8, .1, .1])))


def headlight_field(shape, parameters):
    """Two image-space lamp lobes, independent of paint position and labels.

    Slightly wider/ farther apart toward the bottom approximates perspective;
    this is an RGB augmentation, not measured beam photometry or calibration.
    """
    p = parameters
    h, w = shape
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    xx /= max(1, w - 1); yy /= max(1, h - 1)
    lower = yy - p['cy']
    half_sep = p['separation']/2 + .10*lower
    width = np.clip(p['sx'] + .08*lower, .10, .24)
    left = np.exp(-.5 * (((xx-p['cx']+half_sep)/width)**2 +
                         ((yy-p['cy']-p['vertical_offset'])/p['sy'])**2))
    right = np.exp(-.5 * (((xx-p['cx']-half_sep)/width)**2 +
                          ((yy-p['cy']+p['vertical_offset'])/p['sy'])**2))
    return ((1+p['asymmetry'])*left + (1-p['asymmetry'])*right)/1.15


def apply_night(image, parameters, noise_seed):
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected uint8 BGR image')
    p = parameters
    if p.get('version') != VERSION:
        raise ValueError('Unknown augmentation version')
    rng = np.random.default_rng(noise_seed)
    h, w = image.shape[:2]
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    xx /= max(1, w - 1); yy /= max(1, h - 1)
    # Road illumination has two separate, smoothly overlapping headlamps.
    beam = headlight_field(image.shape[:2], p)
    shadow = 1-p['shadow']*np.exp(-.5*(((xx-p['shadow_x'])/.2)**2+((yy-p['shadow_y'])/.3)**2))
    signal = (image.astype(np.float32)/255.)**p['gamma']
    mean = cv2.GaussianBlur(signal, (0, 0), 9.)
    signal = np.clip(mean+p['contrast']*(signal-mean), 0., 1.)
    signal *= ((p['ambient']+p['beam']*beam)*shadow)[..., None]
    glare = p['glare']*np.exp(-.5*(((xx-p['glare_x'])/.05)**2+((yy-p['glare_y'])/.09)**2))
    signal += glare[..., None]
    signal += rng.normal(size=signal.shape).astype(np.float32)*np.sqrt(
        p['read_noise']**2 + p['shot_noise']*np.maximum(signal, 0.))
    if p['blur'] == 1:
        signal = cv2.GaussianBlur(signal, (3, 3), .6)
    elif p['blur'] == 2:
        kernel = np.ones((1, 3), np.float32)/3
        signal = cv2.filter2D(signal, -1, kernel)
    return np.rint(np.clip(signal, 0., 1.)*255).astype(np.uint8)


def visibility_check(image, label):
    """Quality screen only; NEVER changes image using labels. Review still needed."""
    lane = label == 1
    if lane.sum() < 20:
        return True, dict(lane_pixels=int(lane.sum()), reason='background_or_sparse')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(np.float32)
    contrast = gray-cv2.GaussianBlur(gray, (31, 31), 0)
    brightness = float(np.median(gray[lane]))
    detail = float(np.median(contrast[lane]))
    supported = float(np.mean((gray[lane] >= 8) & (contrast[lane] >= 1.5)))
    return brightness >= 10 and detail >= 2 and supported >= .55, dict(
        lane_pixels=int(lane.sum()), median_lane_gray=brightness,
        median_lane_local_contrast=detail, visible_fraction=supported)


def make_training_view(image, label, severity, seed):
    rng = np.random.default_rng(seed)
    for attempt in range(12):
        parameters = sample_parameters(rng, severity)
        noise_seed = int(rng.integers(0, 2**31))
        out = apply_night(image, parameters, noise_seed)
        good, quality = visibility_check(out, label)
        if good:
            return out, dict(parameters=parameters, noise_seed=noise_seed,
                             attempt=attempt, visibility=quality)
    raise ValueError('NIGHT_VIEW_NOT_OBSERVABLE: inspect label/exposure; no silent clean fallback')


def sequence_parameters(time_s, seed=484, severity='medium'):
    """Stable 2-second illumination knots; sensor noise remains frame-dependent."""
    k = int(np.floor(time_s / 2.))
    a = sample_parameters(np.random.default_rng(seed + k), severity)
    b = sample_parameters(np.random.default_rng(seed + k + 1), severity)
    fraction = (time_s/2.)-k
    blend = fraction*fraction*(3-2*fraction)
    p = {key: ((1-blend)*a[key]+blend*b[key] if isinstance(a[key], float) else a[key])
         for key in a}
    # Avoid unrelated blur toggling in a temporal consistency test.
    p['blur'] = 0
    return p
