import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = 0.5
for k in range(0,5):
    for j in range(0,5):
        for i in range(0,10):
            pyrosim.Send_Cube(name="Box", pos=[x+j, y+k, z+i], size=[(length*(.9**(i+1))), (width*(.9**(i+1))), (height*(.9**(i+1)))])


pyrosim.End()

