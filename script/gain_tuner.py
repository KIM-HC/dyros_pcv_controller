import math

wn = 100

# critically = 1
#      under < 1
#       over > 1
damped = 1.0

kp = wn * wn
kv = int(2.0 * damped * wn)

print('kp:',kp)
print('kv:',kv)

# original gain -> damping_ratio = 0.057735026
# print(math.sqrt(1200))
# print(2.0/math.sqrt(1200))
