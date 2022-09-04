# by carlos 202208
import open3d as o3d
import numpy as np
import os
def pcd2npy(pcd):
    npy=np.asanyarray(pcd.points).astype(np.float32)
    return npy

def get_pcd_size(pcd):
    return pcd2npy(pcd).shape[0]

def pick_points(pcd):
    #http://www.open3d.org/docs/release/python_example/visualization/index.html
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def findAllFile(base):
    filepath=[]
    for root, ds, fs in os.walk(base):
        for f in fs:
            filepath.append(os.path.join(root, f))
    return filepath
    


if __name__=="__main__":
    # file_pcd = "D:/carlos/my_tools/pcd/data/table_scene_lms400.pcd"
    # file_pcd_out = './out.pcd'
    file = open('./pcd_point_pick.txt', mode='w')
    base = './20220825'
    filepaths=findAllFile(base)
    print(filepaths)
    for filename in filepaths:
        print(filename)
        pcd = o3d.io.read_point_cloud(filename)
        # pcd_num=get_pcd_size(pcd)
        # print(pcd_num)
        id_pts = pick_points(pcd)
        file.write(filename+'\n')   
        for i in id_pts:
            print(pcd.points[i])
            file.write(str(pcd.points[i][0])+' '+str(pcd.points[i][1])+' '+str(pcd.points[i][2])+'\n')
        file.write('\n')
    
    file.close()

    # pcd_pts = pcd.select_by_index(id_pts)
    # o3d.io.write_point_cloud(file_pcd_out,pcd_pts)