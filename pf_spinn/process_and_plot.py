from read_dataset import load_spike_train
from matplotlib.patches import Circle
import matplotlib.pyplot as plt

def processAndPlot(video_sequence, output_position):

    frames_to_show = min(len(video_sequence), len(output_position))

    fig, ax = plt.subplots()
    plt.gray()

    for i in range(frames_to_show) :
        ax.cla()
        ax.imshow(video_sequence[i])
        ax.add_patch(Circle((output_position[i][0], output_position[i][1]), output_position[i][2]))

        # plt.show()
        plt.pause(0.01)



# spikes1, video_sequence = load_vbottle(filename="/home/aglover/workspace/datasets/spinnaker_tracking/1/ATIS/data.log", tsscaler=0.000000080)
#
# fig, ax = plt.subplots()
# plt.gray()
# for im in video_sequence :
#     ax.cla()
#     ax.imshow(im)
#     #plt.show()
#     plt.pause(0.01)



# data[256,256] = [255,0,0]
# plt.imshow(data, interpolation='nearest')
# plt.show()