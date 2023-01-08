# Version : V1
# Deadline : 2023 / 01 / 16
# Author : PoLin Jiang
# Discription : HongLang Project

from array import array

class WayPoints:
    def __init__(self, x=0.000, y=0.000, z=0.000, W=0.000, P=0.000, R=0.000, V=100, C='CNT0'):
        self.x = x  # x
        self.y = y  # y
        self.z = z  # z
        self.W = W  # phi
        self.P = P  # theta
        self.R = R  # psi
        self.V = V  # velocity
        self.C = C  # continuity

def point_2_ls(file, waypoints):
    f = open(file, 'w')
    f.write("/PROG  PNS888\n")
    f.write("/ATTR\n")
    f.write("OWNER	= MNEDITOR;\n")
    f.write("COMMENT 	= \"\";\n")
    f.write("PROG_SIZE	= 636\n")
    f.write("CREATE		= DATE 23-01-07  TIME 11:59:14;\n")
    f.write("MODIFIED = DATE 23-01-07  TIME 12:02:18;\n")
    f.write("FILE_NAME	= ;\n")
    f.write("VERSION	= 0;\n")
    f.write("LINE_COUNT	= 4;\n")
    f.write("MEMORY_SIZE	= 992;\n")
    f.write("PROTECT		= READ_WRITE;\n")
    f.write("TCD:  STACK_SIZE	= 0,\n")
    f.write("      TASK_PRIORITY	= 50,\n")
    f.write("      TIME_SLICE	= 0,\n")
    f.write("      BUSY_LAMP_OFF	= 0,\n")
    f.write("      ABORT_REQUEST	= 0,\n")
    f.write("      PAUSE_REQUEST	= 0;\n")
    f.write("DEFAULT_GROUP	= 1,*,*,*,*;\n")
    f.write("CONTROL_CODE	= 00000000 00000000;\n")

    f.write("/MN\n")
    f.write("   1:J P[1] 100% FINE    ;\n")
    for i in range(2,len(waypoints)+1):
        f.write("   " + str(i) + ":L P[" + str(i) + "] " + str(waypoints[i-1].V) + "% " + waypoints[i-1].C + "    ;" + "\n")

    f.write("/POS\n")
    for i in range(1,len(waypoints)+1):
        f.write("P[" + str(i) + "]{\n")
        f.write("    GP1:\n")
        f.write("	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',\n")
        f.write("	X =  " + "{:.3f}".format(waypoints[i-1].x) + "  mm,	Y =   "+ "{:.3f}".format(waypoints[i-1].y) + "  mm,	Z =   "+ "{:.3f}".format(waypoints[i-1].z) + "  mm,\n")
        f.write("	W =  " + "{:.3f}".format(waypoints[i-1].W) + " deg,	P =   " + "{:.3f}".format(waypoints[i-1].P) + " deg,	R =   " + "{:.3f}".format(waypoints[i-1].R) + " deg\n")
        f.write("};\n")
    f.write("/END\n")
    f.close()

def main():
    waypoints = []
    waypoints.append(WayPoints(1320.840,207.050,145.518))
    waypoints.append(WayPoints(1664.839,207.050,145.518))
    waypoints.append(WayPoints(1664.839,-270.050,145.518))
    # print(waypoints[0].x)
    point_2_ls('PNS888.LS',waypoints)

if __name__ == '__main__':
    main()
