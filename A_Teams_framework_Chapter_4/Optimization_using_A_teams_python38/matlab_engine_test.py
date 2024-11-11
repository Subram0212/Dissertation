import matlab.engine
import numpy as np
eng = matlab.engine.start_matlab()

# x = np.array([58, 25, 10])
x = [58, 25, 10]
x = eng.int8(x)
y = eng.sqrt(x[0])
print(y)
