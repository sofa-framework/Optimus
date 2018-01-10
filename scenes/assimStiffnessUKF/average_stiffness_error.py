# load error rate data
errorRate = open('/home/sergei/Source_code/Sofa_development/Optimus/scenes/assimStiffnessUKF/measurements/stiffnessError.txt', 'r')

currentTime = 0.01

timeLine = list()
errorRateLine = list()

# read data from file and parse input string
averageError = 0.0
averagingCoef = 0

for currentLine in errorRate:
    tokens = currentLine.split()
    time = float(tokens(0))
    
    # if needed start new time iteration
    if abs(time - currentTime) > 1e-05:
        timeLine.append(currentTime)
        currentTime = time
        averageError = averageError / averagingCoef
        errorRateLine.append(averageError)
        averageError = 0.0
        averagingCoef = 0
        
    index = 1
    while index < len(tokens):
        if tokens(index) == 'average':
            averageError = averageError + float(tokens(index + 1))
            averagingCoef = averagingCoef + 1
            break

        index = index + 1


# finish last iteration
timeLine.append(currentTime)
currentTime = time
averageError = averageError / averagingCoef;
errorRateLine.append(averageError)


# convert time line to iteration line
for index in range(0, len(timeLine)):
    timeLine(index) = timeLine(index) * 100


# draw results
fig1 = plt.figure(100)
spl1 = fig1.add_subplot(111)
spl1.plot(iterationLine, errorRateLine, linestyle='solid')
spl1.set_title('average stiffness error rate')

plt.show()

 
