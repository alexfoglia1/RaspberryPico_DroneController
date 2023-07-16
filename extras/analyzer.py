import matplotlib
import matplotlib.pyplot as plt
import math
matplotlib.use('TkAgg')

def attitude_est(ax, ay, az, mx, my, mz):
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    heading = 0
    if (my == 0):
        heading = math.pi if (mx < 0) else 0
    else:
        heading = math.atan2(mx, my)

    heading -= 3.75 * math.pi / 180

    if (heading > math.pi):
        heading -= (2 * math.pi)
    elif (heading < -math.pi):
        heading += (2 * math.pi)

    return roll * 180 / math.pi, pitch * 180 / math.pi, heading * 180 / math.pi


lines = []
with open("acquisizione.csv", "r") as f:
    lines = f.readlines()

titles = lines[0].replace(" ", "")
titles = titles.split(",")
valuesdict = {}
for name in titles:
    name = name.replace("\n", "")
    valuesdict[name] = []

for line in lines[1:]:
    line = line.split(",")
    for i in range(len(line)):
        key = titles[i].replace("\n", "")
        value = line[i].replace("\n", "")
        valuesdict[key].append(float(value.replace(" ", "")))

for i in range(len(valuesdict['roll'])):
    valuesdict['roll'][i] *= 180.0/math.pi
    valuesdict['pitch'][i] *= 180.0/math.pi
    valuesdict['yaw'][i] *= 180.0/math.pi

accx_filtered = []
accy_filtered = []
accz_filtered = []
magx_filtered = []
magy_filtered = []
magz_filtered = []
roll_filtered = []
pitch_filtered = []

#WIN_SIZE = 20
#alpha = 0.9
#beta = 1 - alpha
idx = 1
t_interval = 0.01
t_dur = 0.5
accx_filtered.append(valuesdict['accx'][0])
accy_filtered.append(valuesdict['accy'][0])
accz_filtered.append(valuesdict['accz'][0])
while idx < len(valuesdict['accx']): #- WIN_SIZE:
    mvnp1 = valuesdict['accx'][idx]
    mv_pt1n = accx_filtered[idx - 1]
    mv_pt1np1 = mv_pt1n + (mvnp1 - mv_pt1n) * (1 - math.exp(-(t_interval/t_dur)))
    accx_filtered.append(mv_pt1np1)

    mvnp1 = valuesdict['accy'][idx]
    mv_pt1n = accy_filtered[idx - 1]
    mv_pt1np1 = mv_pt1n + (mvnp1 - mv_pt1n) * (1 - math.exp(-(t_interval/t_dur)))
    accy_filtered.append(mv_pt1np1)

    mvnp1 = valuesdict['accz'][idx]
    mv_pt1n = accz_filtered[idx - 1]
    mv_pt1np1 = mv_pt1n + (mvnp1 - mv_pt1n) * (1 - math.exp(-(t_interval/t_dur)))
    accz_filtered.append(mv_pt1np1)


#    accx_filtered.append(accx_filtered[idx -1] * alpha + valuesdict['accx'][idx] * beta)
#    accy_filtered.append(accy_filtered[idx -1] * alpha + valuesdict['accy'][idx] * beta)
#    accz_filtered.append(accz_filtered[idx -1] * alpha + valuesdict['accz'][idx] * beta)
    idx += 1
    #avg_ax = 0
    #avg_ay = 0
    #avg_az = 0
    #avg_mx = 0
    #avg_my = 0
    #avg_mz = 0
    #for i in range(WIN_SIZE):
    #    avg_ax += valuesdict['accx'][idx + i]
    #    avg_ay += valuesdict['accy'][idx + i]
    #    avg_az += valuesdict['accz'][idx + i]
    #    avg_mx += valuesdict['magx'][idx + i]
    #    avg_my += valuesdict['magy'][idx + i]
    #    avg_mz += valuesdict['magz'][idx + i]

    #avg_ax /= WIN_SIZE
    #avg_ay /= WIN_SIZE
    #avg_az /= WIN_SIZE
    #avg_mx /= WIN_SIZE
    #avg_my /= WIN_SIZE
    #avg_mz /= WIN_SIZE

    #for i in range(WIN_SIZE):
    #    accx_filtered.append(avg_ax)
    #    accy_filtered.append(avg_ay)
    #    accz_filtered.append(avg_az)
    #    magx_filtered.append(avg_mx)
    #    magy_filtered.append(avg_my)
    #    magz_filtered.append(avg_mz)

    #idx += WIN_SIZE


roll_filtered = []
pitch_filtered = []

for i in range(len(accx_filtered)):
    ax, ay, az = accx_filtered[i], accy_filtered[i], accz_filtered[i]
    mx, my, mz = valuesdict['magx'][i], valuesdict['magy'][i],valuesdict['magz'][i]

    roll, pitch, heading = attitude_est(ax, ay, az, mx, my, mz)
    roll_filtered.append(roll)
    pitch_filtered.append(pitch)

fig,a = plt.subplots(2, 2)
plt.suptitle('Roll/Pitch analysis')
fig.set_size_inches(12.5, 7)
a[0][0].plot(valuesdict['accx'], '-', label='accx')
a[0][0].plot(accx_filtered, '-', label='accx_filtered')
a[0][0].plot(valuesdict['accy'], '-', label='accy')
a[0][0].plot(accy_filtered, '-', label='accy_filtered')
a[0][0].plot(valuesdict['accz'], '-', label='accz')
a[0][0].plot(accz_filtered, '-', label='accz_filtered')
a[0][0].set_xlabel('Time')
a[0][0].set_ylabel('Accelerometer')
a[0][0].legend()
a[0][1].plot(valuesdict['magx'], '-', label='magx')
a[0][1].plot(magx_filtered, '-', label='magx_filtered')
a[0][1].plot(valuesdict['magy'], '-', label='magy')
a[0][1].plot(magy_filtered, '-', label='magy_filtered')
a[0][1].plot(valuesdict['magz'], '-', label='magz')
a[0][1].plot(magz_filtered, '-', label='magz_filtered')
a[0][1].set_xlabel('Time')
a[0][1].set_ylabel('Magnetomter')
a[0][1].legend()
a[1][0].plot(valuesdict['roll'], '-', label='roll')
a[1][0].plot(roll_filtered, '-', label='roll_filtered')
a[1][0].set_xlabel('Time')
a[1][0].set_ylabel('Roll')
a[1][0].legend()
a[1][1].plot(valuesdict['pitch'], '-', label='pitch')
a[1][1].plot(pitch_filtered, '-', label='pitch_filtered')

a[1][1].set_xlabel('Time')
a[1][1].set_ylabel('Pitch')
a[1][1].legend()

for i in range(0,2):
    for j in range(0,2):
        a[i][j].grid()

plt.show()