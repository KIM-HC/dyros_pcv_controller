import math

wn = 190 #165 #170 #190 #220
# wn_angle = wn * math.sqrt(1100) / math.sqrt(1200)
wn_angle = 165

# critically = 1
#      under < 1
#       over > 1
damped = 1.0
og_damped = 0.057735027
og_angle_damped = 0.030151134457776358

kp = wn * wn
kv = 2.0 * damped * wn
og_kp = wn * wn
og_kv = 2.0 * og_damped * wn
og_kp_angle = wn_angle * wn_angle
og_kv_angle = 2.0 * og_angle_damped * wn_angle


# print('kp:',kp)
# print('kv:',kv)
print('og_kp:',og_kp)
print('og_kv:',og_kv)
print('og_kp_angle:',og_kp_angle)
print('og_kv_angle:',og_kv_angle)
print('')

# ## original gain -> damping_ratio = 0.057735026
# print(math.sqrt(1200))
# print(2.0/math.sqrt(1200))

# ## original angle gain -> damping_ratio = 0.030151134457776358
# print(math.sqrt(1100))
# print(1.0/math.sqrt(1100))
