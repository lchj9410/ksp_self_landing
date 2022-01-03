
# import krpc

# conn = krpc.connect(name='landing')
# vessel = conn.space_center.active_vessel

# vessel.control.gear=True
# vessel.control.toggle_action_group(1)




import time
import math
import numpy as np
from scipy.optimize import fsolve


def my_func(t, *data):
	delta_v, F, m0, g, Isp=data
	return Isp*(np.log(m0)-np.log(m0-F/Isp*t))-g*t-delta_v

ct = time.time()

# delta_v=100
# F=16e3
# m0=1e3
# g=5
# Isp=280*9.82

T0=100/(16e3/1e3-5)

data=(100, 16e3, 1e3, 5, 280*9.82)

ans=fsolve(my_func, T0, args=(100, 16e3, 1e3, 5, 280*9.82))


elapsed = time.time() - ct
print(elapsed+1e-7,ans,my_func(ans[0],*data))

