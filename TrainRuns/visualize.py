### Utilitaries to visualize runs and policy
import os, sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as clrs
import matplotlib.cm as cm
from collections import defaultdict


def visualize_run(run_path, v=40, d=4):
    # Load background
    path = "/home/cvml1/Code/CV/discretized_track_h%d_v%d.png" %(v, d)
    if not os.path.isfile(path):
        print "Error, file %s not found. Continuing without background" %path
        np.zeros((720, 1696, 3), np.uint8)
    else:
        img = cv2.imread(path)
        # Load centroids
        path = "/home/cvml1/Code/CV/centroids_h%d_v%d.txt" %(v, d)
        if not os.path.isfile(path):
            print "Error, file %s not found. Cannot load centroids" %path
            raise SystemExit
        else:
            centroids = {}
            with open(path, "r") as f:
                for i, line in enumerate(f.read().splitlines()):
                    aux = line.split()
                    centroids[i] = (int(float(aux[0])), int(float(aux[1])))

    # Load and parse path
    with open(run_path, "r") as f:
        i = 0
        pos = 1
        for line in f.read().splitlines():
            if line.startswith('State'):
                coords = centroids[int(line.split()[3])]
                #print coords
                cv2.circle(img, (coords[0], coords[1]), 10 if not i else 5, (255, 0, 255), -11)
                i += 1
                if line.startswith('Action'):
                    action = line.split()[2][0].upper()
                    param = line.split()[4]
                    cv2.putText(img, "%d: %s %s" %(i, action, param), (coords[0] - 50, coords[1] + pos * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 1, cv2.LINE_AA)
                    pos *= -1

    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()


def visualize_policy(policy_path, v=40, d=4, joint=False):
    """
    Plot heatmaps for policy visualization.
    
    Args:
     * ``policy_path`` (*str*): Path to file containing the policy (in readable format).
     * ``v`` (*int*): vertical gap parameter used to generate the centroids.
     * ``d`` (*int*): horizontal gap parameter used to generate the centroids.
    """

    # Check if background file exists
    bgpath = "/home/cvml1/Code/CV/discretized_track_h%d_v%d.png" %(v, d)
    if not os.path.isfile(bgpath):
        print >> sys.stderr,  "Error, file %s not found. Continuing without background" %bgpath
        raise SystemExit

    # Check if centroid file exists
    path = "/home/cvml1/Code/CV/centroids_h%d_v%d.txt" %(v, d)
    if not os.path.isfile(path):
        print >> sys.stderr, "Error, file %s not found. Cannot load centroids" %path
        raise SystemExit

    # Load centroids
    centroids = {}
    with open(path, "r") as f:
        for i, line in enumerate(f.read().splitlines()):
            aux = line.split()
            centroids[i] = (int(float(aux[0])), int(float(aux[1])))

    # Parse policy
    heatmaps = [defaultdict(lambda: 0) for _ in xrange(4)]
    with open(policy_path, "r") as f:
        for line in f.read().splitlines():
            line = line.strip()

            if line.startswith('- State'):
                indx = int(line.split()[4])
                speed = int(line.split()[7])

            elif line.startswith('->'):
                action = line.split()[3]
                param = line.split()[5]
                qvalue = float(line.split()[-1]) 

                # Acceleration
                if action == 'speed' and param == '1700' and speed == 1200:
                    heatmaps[0][indx] = qvalue
                # Deceleration
                elif action == 'speed' and param == '1200' and speed == 1700:
                    heatmaps[1][indx] = qvalue
                # Change lane inside [average over speed]
                elif action == 'lane' and  param == 'INSIDE':
                    heatmaps[2][indx] += qvalue
                # Change lane inside [average over speed]
                elif action == 'lane' and  param == 'OUTSIDE':
                    heatmaps[3][indx] += qvalue

        # Plots
        f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
        axes = [ax1, ax2, ax3, ax4]
        cmap = cm.jet
        if joint:
            norm = clrs.Normalize(vmin=min([x for heatmap in heatmaps for x in heatmap.itervalues()]), vmax=max([x for heatmap in heatmaps for x in heatmap.itervalues()]))
            m = cm.ScalarMappable(norm=norm, cmap=cmap)
            
        for k, heatmap in enumerate(heatmaps):
            # Normalization
            if not joint:
                norm = clrs.Normalize(vmin=min(heatmap.itervalues()), vmax=max(heatmap.itervalues()))
                m = cm.ScalarMappable(norm=norm, cmap=cmap)
            img = cv2.imread(bgpath)

            # heatmap
            for i, v in heatmap.iteritems():
                coords = centroids[i]
                cv2.circle(img, (coords[0], coords[1]), 12, map(lambda x: int(x * 255), m.to_rgba(v)), -11)
            #axes[k].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            axes[k].imshow(img)
            axes[k].set_adjustable('box-forced')

    axes[0].set_title('Acceleration Heatmap')
    axes[1].set_title('Decelaration heatmap')
    axes[2].set_title('Change to inside lane Heatmap')
    axes[3].set_title('Change to outside lane Heatmap')
    f.tight_layout() 
    if joint:
        f.savefig('heatmap_%s_normalized.pdf' % policy_path.rsplit('.', 1)[0].rsplit('/', 1)[1], bbox_inches='tight', dpi=500)
    else:
        f.savefig('heatmap_%s.pdf' % policy_path.rsplit('.', 1)[0].rsplit('/', 1)[1], bbox_inches='tight', dpi=500)
    plt.show()


        
if __name__ == "__main__":
    #visualize_run(sys.argv[1])
    if len(sys.argv) < 3:
        print >> sys.stderr, "Usage: ./visualize [policy|run] [path]"
        raise SystemExit
    if sys.argv[1] == 'run':
        visualize_run(sys.argv[2])
    elif sys.argv[1] == 'policy':
        visualize_policy(sys.argv[2], joint=(len(sys.argv) > 3))
    else:
        print >> sys.stderr, "Unknown mode. Allowed: 'policy' and 'run'"
