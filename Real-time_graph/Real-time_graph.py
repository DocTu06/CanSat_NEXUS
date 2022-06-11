import matplotlib.pyplot as plt
import serial

def isfloat(input):
    try:
        float(input)
        return True
    except:
        return False
comgood = 0
file = open('LoRa.txt', "a")
while comgood == 0:

    try:
        port = input('Type the COM port: ')
        COMport = 'COM' + str(port)
        ser = serial.Serial(COMport, 9600)
        comgood = 1
    except:
        print("Wrong port. Insert another port.")

temp = []
hum = []
prs = []
air = []
ser.close()
ser.open()
data = ser.readline()
input = data.decode()
file.write(input)
file.close()
numbers = input.split(',')
if len(numbers) == 7:
    if isfloat(numbers[0]):
        temp = [float(numbers[0])]
    if isfloat(numbers[1]):
        prs = [float(numbers[1])]
    if isfloat(numbers[2]):
        hum = [float(numbers[2])]
    if isfloat(numbers[5]):
        air = [float(numbers[5])]
plt.ion()
figure, axis = plt.subplots(2, 2)
while 1:

    data = ser.readline()
    input = data.decode()
    file = open('LoRa.txt', "a")
    file.write(input)
    print(input)
    file.close()
    numbers = input.split(',')
    if len(numbers) == 7:
        if isfloat(numbers[0]):
            temp.append(float(numbers[0]))
        if isfloat(numbers[1]):
            prs.append(float(numbers[1]))
        if isfloat(numbers[2]):
            hum.append(float(numbers[2]))
        if isfloat(numbers[5]):
            air.append(float(numbers[5]))
        axis[0,0].plot(temp,color = 'red')
        axis[1,0].plot(prs,color = 'orange')
        axis[0,1].plot(hum,color = 'blue')
        axis[1,1].plot(air,color = 'green')
        figure.canvas.draw()
        figure.canvas.flush_events()
        plt.pause(0.1)
        axis[0,0].cla()
        axis[0,1].cla()
        axis[1,0].cla()
        axis[1,1].cla()
