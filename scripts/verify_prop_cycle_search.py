"""Long-period detection and rejection of drifting native-output surrogates."""
import numpy as np
from prop_cycle import aligned_window,_aligned_window,aircraft_window


def main():
    period=257;frames=np.arange(6*period,dtype=float)
    baseline=np.array([1000.]*9+[10.,10.]);amplitude=np.array([100.]*9+[1.,1.])
    outputs=baseline+amplitude*np.sin(2*np.pi*frames[:,None]/period+np.arange(11)[None,:]*.19)
    # The previous warm-start horizon cannot see this measured period.
    assert _aligned_window(outputs.tolist(),96,12) is None
    result=aligned_window(outputs.tolist(),96,12)
    assert result and result['frames']==period,result
    assert max(result['changes'])<=7e-4 and result['envelope']<=1e-3 and result['waveform']<=5e-3
    drifting=outputs.copy();drifting[:,4]+=frames*.1
    assert aligned_window(drifting.tolist(),96,12) is None,'A drifting output channel was accepted'
    assert aligned_window(outputs[:200].tolist(),96,12) is None,'Insufficient evidence was accepted'
    # A short native period still uses the shortest directly certified cycle.
    short=baseline+amplitude*np.sin(2*np.pi*np.arange(600)[:,None]/24+np.arange(11)[None,:]*.19)
    assert aligned_window(short.tolist(),96,12)['frames']==24
    rng=np.random.default_rng(2319)
    noise=np.zeros((2880,11));noise[:,0]=9000.+rng.normal(0,2,2880)
    noise[:,4]=-40.+rng.normal(0,8,2880);noise[:,9]=2.
    physical=aircraft_window(noise,480,64000.,[50000.,95000.,44000.])
    assert physical and physical['force_mean_uncertainty_g']<=1e-4
    assert max(physical['angular_mean_uncertainty_rad_s2'])<=2.5e-5
    drift=noise.copy();drift[:,4]+=np.arange(len(drift))*.1
    assert aircraft_window(drift,480,64000.,[50000.,95000.,44000.]) is None
    wash=noise.copy();wash[:,9]+=np.arange(len(wash))*.001
    assert aircraft_window(wash,480,64000.,[50000.,95000.,44000.]) is None
    assert aircraft_window(noise[:800],480,64000.,[50000.,95000.,44000.]) is None
    print('PASS: long/short periods, all-channel drift rejection, residual-scaled aperiodic mean, minimum evidence')


if __name__=='__main__':main()
