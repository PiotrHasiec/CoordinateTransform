import pandas as pd
import matplotlib.pyplot as plt


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
data = pd.read_csv('Dron1Dron2LotdowolnyT01BBox.csv')
prew = []
colors = {0:"red",3:"blue",6:"black"}
for i in range(data.shape[1]):
    prew.append([])
    
# for i,row in data.iterrows():
#     if i < 10:
#            continue
#     for i,d in enumerate(row):
#             prew[i] = d
#     break

for i,row in data.iterrows() :
    
    if i%2 == 1 and i <5000 and i>2000:
        ax.clear()
   
        ax.set_xlim(-2000,2000)
        ax.set_ylim(-2000,2000)
        ax.set_zlim(0,1600)
        for j in range(0,len(row),3):
            
            if True:#not j in [0,3,9,12,15,18]:
                # ax.scatter([row[j],prew[j]],[row[j+1],prew[j+1]],[row[j+2],prew[j+2]])
                ax.scatter([row[j]],[row[j+1]],[row[j+2]])
                prew[j] = row[j]
                prew[j+1] = row[j+1]
                prew[j+2] = row[j+2]
        # plt.show()    
        plt.pause(0.001)
        print(i)
plt.show()