import math

wn = 100

# critically = 1
#      under < 1
#       over > 1
damped = 1.0
og_damped = 0.057735027

kp = wn * wn
kv = int(2.0 * damped * wn)
og_kp = wn * wn
og_kv = int(2.0 * og_damped * wn)

print('kp:',kp)
print('kv:',kv)
print('og_kp:',og_kp)
print('og_kv:',og_kv)
print('')

# original gain -> damping_ratio = 0.057735026
# print(math.sqrt(1200))
# print(2.0/math.sqrt(1200))
