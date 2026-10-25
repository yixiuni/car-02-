import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from calibrate_gray import fit

white=[(i,100,100,100,100,900,900,900,900) for i in range(128)]
black=[(i,900,900,900,900,100,100,100,100) for i in range(128)]
channels=fit(white,black)
assert all(c['threshold']==500 for c in channels)
assert [c['black_high'] for c in channels]==[True]*4+[False]*4
for bad in [white,[(i,900,900,900,100,100,100,100,100) for i in range(128)],
            [(i,900 if i else 99,900,900,900,100,100,100,100) for i in range(128)]]:
    try:fit(white,bad)
    except ValueError:pass
    else:raise AssertionError('Ambiguous/noisy training data accepted')
print('PASS: threshold fitting rejects overlap/noise and handles both polarities')
