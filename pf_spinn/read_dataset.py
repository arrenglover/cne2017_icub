
def load_spike_train(filename):

    file = open(filename, "r")

    spike_train = [[]] * 1048575

    for line in file:
        line_split = line.split(";")
        index = int(line_split[0]) + (int(line_split[1]) << 9)

        line_split[2] = line_split[2][:-1]
        if line_split[2]:
            times = line_split[2].split(",")

            times = map(int, times)

            spike_train[index] = times

    file.close()

    return spike_train