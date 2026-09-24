"""Choose whole output cycles for the existing native mean certificate."""
import numpy as np


def aircraft_window(outputs,window,weight,inertia,torque_gyro=True):
    """Bound mean uncertainty in the aircraft's force/moment error units.

    Opposed engines can have tiny net moments with finite native governor
    fluctuations. A relative test on that cancelled net moment asks for
    precision unrelated to aircraft balance. Keep the original checks for
    wash/angular momentum, and explicitly bound force and moment uncertainty
    to half of the existing aircraft residual tolerances. The final aircraft
    check adds this uncertainty to its residual before applying those limits.
    """
    values=np.asarray(outputs,dtype=float)
    for length in dict.fromkeys((window,2*window,len(values)//3)):
        if length<window or len(values)<3*length:continue
        blocks=values[-3*length:].reshape(3,length,11);means=blocks.mean(axis=1)
        scales=np.maximum([100.]*9+[1.]*2,np.max(abs(means),axis=0))
        changes=abs(np.diff(means,axis=0))
        envelope=np.maximum(abs(blocks[2].min(0)-blocks[1].min(0)),
                            abs(blocks[2].max(0)-blocks[1].max(0)))
        # Aperiodic native governor noise has sample-dependent extrema.
        # Check the stability of its RMS amplitude, while retaining the
        # observed extrema as diagnostics. Mean drift is checked separately.
        rms_change=np.max(abs(np.diff(blocks.std(axis=1),axis=0)),axis=0)
        force=float(np.max(np.linalg.norm(changes[:,:3],axis=1))/weight)
        angular=np.max(changes[:,3:6],axis=0)/np.asarray(inertia)
        if (force<=1e-4 and np.max(angular)<=2.5e-5 and
                np.linalg.norm(rms_change[:3])/weight<=2e-4 and
                np.max(rms_change[3:6]/np.asarray(inertia))<=5e-5 and
                np.max(changes[:,6:(11 if torque_gyro else 10)]/scales[6:(11 if torque_gyro else 10)])<=7e-4 and
                np.max(envelope[6:(11 if torque_gyro else 10)]/scales[6:(11 if torque_gyro else 10)])<=1e-3):
            return dict(frames=length,force_mean_uncertainty_g=force,
                        angular_mean_uncertainty_rad_s2=angular.tolist(),
                        changes=(changes/scales).max(axis=1).tolist(),
                        spread=np.ptp(blocks[2],axis=0).tolist(),
                        rms_relative_change=(rms_change/scales).tolist(),
                        envelope=float(np.max(envelope/scales)))
    return None


def aligned_window(outputs, window, minimum_period):
    """Find a recurring output phase; retain only actual consecutive frames.

    Fixed wall-time windows can cut a steady governor oscillation at different
    phases. Their means then alternate forever. A recurrence proposes an
    integer window length; all output channels must subsequently pass the
    unchanged mean/envelope checks, plus a waveform recurrence check.
    """
    if window<minimum_period or len(outputs)<3*window:return None
    # A continuation window is a minimum observation length, not a maximum
    # physical governor period. Search longer observed cycles as evidence
    # accumulates. Otherwise a stable five-second cycle can fail forever when
    # every warm-start test only considers periods shorter than two seconds.
    for horizon in dict.fromkeys((window,len(outputs)//3)):
        result=_aligned_window(outputs,horizon,minimum_period)
        if result is not None:return result
    return None


def _aligned_window(outputs, window, minimum_period):
    values=np.asarray(outputs[-3*window:],dtype=float)
    scale=np.maximum([100.]*9+[1.]*2,np.max(abs(values),axis=0))
    channel=int(np.argmax(np.ptp(values[-window:],axis=0)/scale))
    signal=values[:,channel]/scale[channel]
    lags=np.arange(minimum_period,window+1)
    # Compute every lag's squared distance by correlation, in O(N log N)
    # work and O(N) memory. Correlation only proposes periods: all eleven
    # output channels still face the original direct certificate below.
    recent=signal[-window:]
    size=1<<(len(signal)+window-2).bit_length()
    correlation=np.fft.irfft(np.fft.rfft(signal,size)*np.fft.rfft(recent[::-1],size),size)
    prefix=np.r_[0.,np.cumsum(signal*signal)]
    starts=2*window-lags
    errors=np.maximum(0.,(np.dot(recent,recent)+prefix[starts+window]-prefix[starts]
                         -2*correlation[starts+window-1])/window)
    # Prefer the shortest recurring phase, rather than a long near-exact
    # harmonic that needlessly multiplies nonlinear aircraft phase replays.
    minima=(errors<=np.r_[np.inf,errors[:-1]])&(errors<=np.r_[errors[1:],np.inf])
    candidates=np.flatnonzero(minima&(errors<=5e-3**2))
    # Ranking is a search only. Drift in ANY component rejects the candidate.
    for index in candidates:
        period=int(lags[index]);blocks=values[-3*period:].reshape(3,period,11)
        means=blocks.mean(axis=1)
        scales=np.maximum([100.]*9+[1.]*2,np.max(abs(means),axis=0))
        changes=np.max(abs(np.diff(means,axis=0))/scales,axis=1)
        minima=blocks.min(axis=1);maxima=blocks.max(axis=1)
        envelope=float(np.max(np.maximum(abs(minima[2]-minima[1]),abs(maxima[2]-maxima[1]))/scales))
        waveform=float(np.max(abs(np.diff(blocks,axis=0))/scales))
        if np.max(changes)<=7e-4 and envelope<=1e-3 and waveform<=5e-3:
            return dict(frames=period,changes=changes.tolist(),envelope=envelope,
                        spread=(maxima[2]-minima[2]).tolist(),waveform=waveform)
    return None
