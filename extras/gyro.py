import matplotlib.pyplot as plt

def mean(v):
    return sum(v)/len(v)

values = []
with open("gyro_bias_z.txt", "r") as f:
    values = f.readlines()

values = [float(value.replace("\n", "")) for value in values]
print(mean(values))

plt.hist(values)
plt.show()