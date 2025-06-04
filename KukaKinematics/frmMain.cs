using Pastel;
using System.Runtime.InteropServices;
using static KukaKinematics.frmMain;

namespace KukaKinematics
{



    public partial class frmMain : Form
    {
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool AllocConsole();
        public class Joints
        {
            public double J1, J2, J3, J4, J5, J6;
            public Joints()
            {
                J1 = 0;
                J2 = 0;
                J3 = 0;
                J4 = 0;
                J5 = 0;
                J6 = 0;
            }
        }

        public class KUKAHR60
        {
            cTransform baseTransform = new (0, 0, 0, 0, 0, 0, 0);
            cVector3d J6ToEECG = new (-1.91, -92.53, -227.89);
            public double eeMass = 36.77; // KG
            public cVector3d EETestMassCG = new(-16*25.4, 0, 0); // Test mass at end effector
            public double testMass = 45 * 0.45359237;


            List<cTransform> transforms = new List<cTransform>();
            public Joints joints = new Joints();

            public KUKAHR60()
            {

                //world coordinate is Z up, Y facing away from operator stand, X to the right
                // robot at 0,0,0 facing toward operator stand, mostr link offsets are in the negative y direction
                double j1yOff = -350;
                double j2yOff = -850;
                double j3yOff = -820;
                double j3zoff = 145;
                double j5yoff = -170;

                // remember, LHTs are translate then orientate. 
                // so the revlute axies goes in the transform before the link length offset

                transforms.Add(new cTransform(0, 0, 0, 0, 180, 0, 0)); //J1
                // world.y = -y, world.z = -z, world.x = x
                transforms.Add(new cTransform(1, 0, -j1yOff, 0, -180, -90, 0)); // J2
                // world.y = y, world.z = x, world.x = -z
                transforms.Add(new cTransform(2, 0, j2yOff, 0, 0, 0, 0)); // J3
                // world.y = y, world.z = x, world.x = -z
                transforms.Add(new cTransform(3, j3zoff, j3yOff, 0, 90, 0, 0)); // J4 
                // world.y = -z, world.z = x, world.x = -y
                transforms.Add(new cTransform(4, 0, 0, 0, -90, 0, 0)); // J5
                // world.y = y, world.z = x, world.x = -z
                transforms.Add(new cTransform(5, 0, j5yoff, 0, -90, 0, 0)); // J6
                transforms.Add(new cTransform(6, 0, 0, 0, 0, 90, 90)); // orient back to world
            }

            public cPose getPoseAtFlange(Joints joints)
            {
                transforms[0].rz = joints.J1;
                transforms[1].rz = joints.J2;
                transforms[2].rz = joints.J3;
                transforms[3].rz = joints.J4;
                transforms[4].rz = joints.J5;
                transforms[5].rz = joints.J6;

                // Calculate the pose by multiplying the transforms
                cLHT lht = new cLHT(); // Start with the base transform

                for( int i = 0; i < transforms.Count; i++)
                {
                    lht = lht * new cLHT(transforms[i]);
                }

                return lht.getPoseEulerXYZ();
            }

            public cPose getJointLocation(int jointIndex, Joints joints)
            {
                if (jointIndex < 0 || jointIndex >= transforms.Count)
                    throw new ArgumentOutOfRangeException(nameof(jointIndex), "Joint index is out of range.");
                // Set the joint angle for the specified joint
                transforms[jointIndex].rz = jointIndex switch
                {
                    0 => joints.J1,
                    1 => joints.J2,
                    2 => joints.J3,
                    3 => joints.J4,
                    4 => joints.J5,
                    5 => joints.J6,
                    _ => throw new ArgumentOutOfRangeException(nameof(jointIndex), "Invalid joint index.")
                };
                // Calculate the pose by multiplying the transforms up to the specified joint
                cLHT lht = new cLHT(); // Start with the base transform
                for (int i = 0; i <= jointIndex; i++)
                {
                    lht = lht * new cLHT(transforms[i]);
                }
                return lht.getPoseEulerXYZ();
            }

            public cPose getPoseAtFlange()
            {
                return getPoseAtFlange(joints);
            }

            public cPose getPoseAtCOM()
            {
                return getPoseAtCOM(joints);
            } 

            public cPose getPoseAtCOM(Joints joints)
            {
                cLHT lhtAtFlange = getPoseAtFlange(joints).getLHT();

                cVectorDouble vCOM = J6ToEECG.v * eeMass + EETestMassCG.v * testMass;
                vCOM = vCOM / (eeMass + testMass); // Average the COM based on the masses

                cTransform resultantCG = new cTransform(vCOM[0], vCOM[1], vCOM[2], 0, 0, 0); // Transform from J6 to end effector CG
                cLHT lhtCOM = new cLHT(resultantCG);
                lhtCOM = lhtAtFlange * lhtCOM;
                return lhtCOM.getPoseEulerXYZ();
            }
        }
        public frmMain()
        {
            InitializeComponent();
            AllocConsole();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            
        }

        private void frmMain_Load(object sender, EventArgs e)
        {
            KUKAHR60 myRobot = new KUKAHR60();
            cPose flangeInWorld = myRobot.getPoseAtFlange();
            cPose COM_World = myRobot.getPoseAtCOM();

            // Assume lhtJ6 is your Left-Handed Transform from J6 frame to world
            cLHT lhtJ6 = myRobot.getJointLocation(5, myRobot.joints).getLHT(); // Get the LHT for joint 6

            // 1. Extract k (Z-axis of joint 6)
            cVector3d z6_World = new cVector3d(lhtJ6.M[0, 2], lhtJ6.M[1, 2], lhtJ6.M[2, 2]);
            Console.WriteLine($"z6_World: {z6_World.v.GetMagnitude():F3}, vector: {z6_World.v[0]:F3}, {z6_World.v[1]:F3}, {z6_World.v[2]:F3}".Pastel(Color.Cyan));

            // 2. Extract R (origin of joint 6 in world)
            cVector3d joint6Origin_World = new cVector3d(lhtJ6.M[0, 3], lhtJ6.M[1, 3], lhtJ6.M[2, 3]);

            cVector3d vCOM = new cVector3d(COM_World.x, COM_World.y, COM_World.z);

            // 3. Compute moment arm from joint to COM
            cVectorDouble r = vCOM.v - joint6Origin_World.v; // both in world

            Console.WriteLine($"Moment arm from J6 to COM: mag: {r.GetMagnitude():F3}, vector: {r[0]:F3}, {r[1]:F3}, {r[2]:F3}".Pastel(Color.Cyan));

            // 4. Compute force due to gravity
            cVector3d gravity = new cVector3d(0, 0 ,-9.81);

            double toolMass = myRobot.eeMass + myRobot.testMass; // mass of the end effector in kg
            cVectorDouble force = toolMass * gravity.v;

            Console.WriteLine($"force: mag: {force.GetMagnitude():F3}, vector: {force[0]:F3}, {force[1]:F3}, {force[2]:F3}".Pastel(Color.Cyan));

            // 5. Compute torque vector in world frame
            cVectorDouble torque_World = r.Cross(force);
            Console.WriteLine($"Torque: mag: {torque_World.GetMagnitude():F3}, vector: {torque_World[0]:F3}, {torque_World[1]:F3}, {torque_World[2]:F3}");

            // 6. Project torque onto joint axis
            double torqueAboutJ6 = torque_World * z6_World.v; //torque_World.Dot(z6_World);

            // 7. Output result
            Console.WriteLine($"Torque about J6 (Nm): {torqueAboutJ6:F2}");


            OutputPoseToConsole(myRobot.joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(myRobot.joints, COM_World, "COM in World");

            Joints joints = new Joints();
            joints.J1 = 0;
            joints.J2 = 90;
            joints.J3 = -90;
            joints.J4 = 0;
            joints.J5 = 0;
            joints.J6 = 0;

            flangeInWorld = myRobot.getPoseAtFlange(joints);
            COM_World = myRobot.getPoseAtCOM(joints);
            OutputPoseToConsole(joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(joints, COM_World, "COM in World");


            joints.J1 = -45;
            joints.J2 = 90;
            joints.J3 = -90;
            joints.J4 = 0;
            joints.J5 = 0;
            joints.J6 = 0;

            flangeInWorld = myRobot.getPoseAtFlange(joints);
            COM_World = myRobot.getPoseAtCOM(joints);
            OutputPoseToConsole(joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(joints, COM_World, "COM in World");



            joints.J1 = 0;
            joints.J2 = 90;
            joints.J3 = -90;
            joints.J4 = 0;
            joints.J5 = 45;
            joints.J6 = 0;

            flangeInWorld = myRobot.getPoseAtFlange(joints);
            OutputPoseToConsole(joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(joints, COM_World, "COM in World");



            joints.J1 = 0;
            joints.J2 = 90;
            joints.J3 = -90;
            joints.J4 = 45;
            joints.J5 = 0;
            joints.J6 = 0;

            flangeInWorld = myRobot.getPoseAtFlange(joints);
            COM_World = myRobot.getPoseAtCOM(joints);
            OutputPoseToConsole(joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(joints, COM_World, "COM in World");


            joints.J1 = 0;
            joints.J2 = 90;
            joints.J3 = -90;
            joints.J4 = 0;
            joints.J5 = 0;
            joints.J6 = 45;

            flangeInWorld = myRobot.getPoseAtFlange(joints);
            COM_World = myRobot.getPoseAtCOM(joints);
            OutputPoseToConsole(joints, flangeInWorld, "Pose at Flange In World");
            OutputPoseToConsole(joints, COM_World, "COM in World");

        }

        public void OutputPoseToConsole(Joints joints, cPose pose,string title)
        {
            Console.WriteLine();
            Console.WriteLine(title);
            Console.WriteLine($"J1: {joints.J1:F0} J2: {joints.J2:F0} J3: {joints.J3:F0} J4: {joints.J4:F0} J5: {joints.J5:F0} J6: {joints.J6:F0}".Pastel(Color.Yellow));
            Console.WriteLine($"X: {pose.x:F3}".Pastel(Color.Green));
            Console.WriteLine($"Y: {pose.y:F3}".Pastel(Color.Green));
            Console.WriteLine($"Z: {pose.z:F3}".Pastel(Color.Green));
            Console.WriteLine($"RX: {pose.rx:F3}".Pastel(Color.Green));
            Console.WriteLine($"RY: {pose.ry:F3}".Pastel(Color.Green));
            Console.WriteLine($"RZ: {pose.rz:F3}".Pastel(Color.Green));
        }
    }
}
