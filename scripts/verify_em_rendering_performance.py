"""Compare batched rendering to the previous individual-cell algorithm."""
import importlib.util,json,time
from pathlib import Path
import numpy as np
from em_plot import enrich,heatmap_column

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/performance-pass'


def main():
    rng=np.random.default_rng(2049);failures=[]
    for case in range(80):
        y=np.cumsum(rng.random(601));z=rng.normal(size=601);mask=rng.random(601)<.08
        if case%10==0:y[23]=y[22]  # Retain legacy handling for diagnostic data.
        grid=np.unique(np.r_[np.linspace(-1.,y[-1]+1,401),y[::37]])
        old=np.full(len(grid),np.nan)
        for j in range(len(y)-1):
            if mask[j:j+2].any():continue
            select=(grid>=y[j])&(grid<=y[j+1])
            old[select]=np.interp(grid[select],y[j:j+2],z[j:j+2])
        new=heatmap_column(y,z,np.where(~mask)[0],grid)
        if not np.array_equal(old,new,equal_nan=True):failures.append(dict(stage='masked interpolation',case=case))
    # A complete saved aircraft chart checks contours, bounds and all displayed
    # heatmap cells; use the same numerical data for both rendering paths.
    baseline=OUT/'baseline/em_plot.py'
    spec=importlib.util.spec_from_file_location('previous_em_plot',baseline)
    old_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(old_module)
    file=OUT/'validated-full-0.json';times=[];charts=[]
    for render in [old_module.enrich,enrich]:
        data=json.loads(file.read_text());start=time.monotonic();render(data);times.append(time.monotonic()-start)
        a=data['aircraft'][0];charts.append({k:a[k] for k in ['heatmap','contours','boundary','sustained_curve','numerical_gaps','numerical_boundaries']})
    identical=charts[0]==charts[1]
    if not identical:failures.append(dict(stage='complete chart changed'))
    report=dict(masked_cases=80,complete_chart_identical=identical,
                previous_seconds=times[0],current_seconds=times[1],failures=failures)
    (OUT/'rendering-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True);assert not failures,failures


if __name__=='__main__':main()
