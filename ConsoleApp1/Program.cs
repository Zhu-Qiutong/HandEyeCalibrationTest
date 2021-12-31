using System;
using System.Data;
using System.Windows;
using Microsoft.VisualBasic.FileIO;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.Data.Text;
using OpenCvSharp;


namespace ConsoleApplication4
{
    public static class Program
    {
        private static void Main()
        {
            // ----------------- read PoseNew2 -----------------
            var path = @".\\Data\1009_PoseNew2.csv";
            Matrix<double> PoseNew2 = DelimitedReader.Read<double>(path, false, ",", false);

            //Console.WriteLine(PoseNew2);
            int n_Col = PoseNew2.ColumnCount;
            int n_Row = PoseNew2.RowCount;

            // Extract desired submatrix
            var trans16 = PoseNew2.SubMatrix(0, n_Row, 0, 3); // Get trans16 (25x3), number of rows depends on the number of photos taken
            var quat16 = PoseNew2.SubMatrix(0, n_Row, 3, n_Col - 3); // Get quat16 (25x4)

            // ----------------- read TranslNew2 -----------------
            path = @".\\Data\1009_TranslNew2.csv";
            Matrix<double> TranslNew2 = DelimitedReader.Read<double>(path, false, ",", true);

            //Console.WriteLine(TranslNew2);
            n_Col = TranslNew2.ColumnCount;
            n_Row = TranslNew2.RowCount;

            // Extract desired submatrix
            Matrix<double> transVar = TranslNew2.SubMatrix(0, n_Row, 1, n_Col - 1); // Get transVar (25x3)

            // ----------------- read RotNew2 -----------------
            path = @".\\Data\1009_RotNew2.csv";
            Matrix<double> RotNew2 = DelimitedReader.Read<double>(path, false, ",", true);

            n_Col = RotNew2.ColumnCount;
            n_Row = RotNew2.RowCount;
        
            // Extract desired submatrix
            var rotVars = RotNew2.SubMatrix(0, n_Row, 1, n_Col - 1); // Get rotVars (25x3)

            // ----------------- read RotNew2 -----------------
            path = @".\\Data\eulZYX.csv";
            Matrix<double> EluNew2 = DelimitedReader.Read<double>(path, false, ",", false);


            /////////////////////////////  R_gripper2base  /////////////////////////////////

            List<Mat> R_gripper2base = new List<Mat>(); // input to CalibrateCamera

            // Create List object to store temp values
            //for (int i = 0; i < 25; i++)
            //{
            //    Mat Temp_Euler = new Mat(3, 1, MatType.CV_64F);
            //    for (int j = 0; j < 3; j++)
            //    {
            //        Temp_Euler.At<double>(j, 0) = EluNew2[i, j];
            //    }
            //    R_gripper2base.Add(Temp_Euler);
            //}
            for (int i = 0; i < 25; i++)
            {
                Mat Temp_Quat = new Mat(4, 1, MatType.CV_64F);
                for (int j = 0; j < 4; j++)
                {
                    Temp_Quat.At<double>(j, 0) = quat16[i, j];
                }
                Mat Temp_Rvector_gripper2base = new Mat(4, 1, MatType.CV_64F);
                Mat RotationVector = new Mat();
                //Mat RodInputMat = new Mat();
                Mat RodInputMat = q2m(Temp_Quat.At<double>(0, 0), Temp_Quat.At<double>(1, 0), Temp_Quat.At<double>(2, 0), Temp_Quat.At<double>(3, 0));
                
                
                Cv2.Rodrigues(RodInputMat, RotationVector);
                //Console.WriteLine(i);
                //printMat(RotationVector);
                //printMat(RotationVector);
                //Temp_Rvector_gripper2base.At<double>(j, 0) = quat16[i, j];
                R_gripper2base.Add(RotationVector);
                //Console.WriteLine(i);
                //printMat(RotationVector);

            }
            //printMat(R_gripper2base[1]);

            /////////////////////////////  t_gripper2base  /////////////////////////////////

            List<Mat> t_gripper2base = new List<Mat>(); // input to CalibrateCamera
            // Create List object to store temp values
            for (int i = 0; i < 25; i++)
            {
                Mat temp_t_gripper2base = new Mat(3, 1, MatType.CV_64F);
                for (int j = 0; j < 3; j++)
                {
                    temp_t_gripper2base.At<double>(j, 0) = trans16[i, j];
                }
                t_gripper2base.Add(temp_t_gripper2base);
            }

            //////////////////////////////  R_target2cam  ////////////////////////////////////

            List<Mat> R_target2cam = new List<Mat>(); // input to CalibrateCamera
            // Create List object to store temp values
            for (int i = 0; i < 25; i++)
            {
                Mat temp_R_target2cam = new Mat(3, 1, MatType.CV_64F);
                for (int j = 0; j < 3; j++)
                {
                    temp_R_target2cam.At<double>(j, 0) = rotVars[i, j];
                }
                R_target2cam.Add(temp_R_target2cam);
            }


            //////////////////////////////  R_target2cam  ////////////////////////////////////
            List<Mat> t_target2cam = new List<Mat>(); // input to CalibrateCamera
            // Create List object to store temp values
            for (int i = 0; i < 25; i++)
            {
                Mat temp_t_target2cam = new Mat(3, 1, MatType.CV_64F);
                for (int j = 0; j < 3; j++)
                {
                    temp_t_target2cam.At<double>(j, 0) = transVar[i, j];
                }
                t_target2cam.Add(temp_t_target2cam);
            }

            Mat rvecs = new(); // output from CalibrateCamera
            Mat tvecs = new(); // output from CalibrateCamera

            Cv2.CalibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, OutputArray.Create(rvecs), OutputArray.Create(tvecs));
            printMat(tvecs);
            printMat(rvecs);
        }


        //public static Mat ToEulerAngles(Mat q)      //  qw qx qy qz
        //{
        //    Mat angles = new Mat(1, 3, MatType.CV_64F);

        //    // roll (x-axis rotation)
        //    double sinr_cosp = 2 * (q.At<double>(0, 0) * q.At<double>(0, 1) + q.At<double>(0, 2) * q.At<double>(0, 3));
        //    double cosr_cosp = 1 - 2 * (q.At<double>(0, 1) * q.At<double>(0, 1) + q.At<double>(0, 2) * q.At<double>(0, 2));
        //    angles.At<double>(0, 0) = Math.Atan2(sinr_cosp, cosr_cosp);

        //    // pitch (y-axis rotation)
        //    double sinp = 2 * (q.At<double>(0, 0) * q.At<double>(0, 2) - q.At<double>(0, 3) * q.At<double>(0, 1));
        //    if (Math.Abs(sinp) >= 1)
        //    {
        //        angles.At<double>(0, 1) = Math.CopySign(Math.PI / 2, sinp);
        //    }
        //    else
        //    {
        //        angles.At<double>(0, 1) = Math.Asin(sinp);
        //    }

        //    // yaw (z-axis rotation)
        //    double siny_cosp = 2 * (q.At<double>(0, 0) * q.At<double>(0, 3) + q.At<double>(0, 1) * q.At<double>(0, 2));
        //    double cosy_cosp = 1 - 2 * (q.At<double>(0, 2) * q.At<double>(0, 2) + q.At<double>(0, 3) * q.At<double>(0, 3));
        //    angles.At<double>(0, 2) = Math.Atan2(siny_cosp, cosy_cosp);

        //    return angles;
        //}


        public static void printMat(Mat M) // print mat results
        {
            int n_row = M.Rows;
            int n_col = M.Cols;

            for (int i = 0; i < n_row; i++)
            {
                for (int j = 0; j < n_col; j++)
                {
                    Console.Write(M.At<double>(i, j));
                    Console.Write(" ");
                }
                Console.WriteLine(";");
            }
        }

        private static Mat q2m(double a, double b, double c, double d)
        {
            Mat Rotm = new Mat(3, 3, MatType.CV_64F);
            Rotm.At<double>(0, 0) = 2 * (a * a + b * b) - 1;
            Rotm.At<double>(0, 1) = 2 * (b * c - a * d);
            Rotm.At<double>(0, 2) = 2 * (b * d + a * c);
            Rotm.At<double>(1, 0) = 2 * (b * c + a * d);
            Rotm.At<double>(1, 1) = 2 * (a * a + c * c) - 1;
            Rotm.At<double>(1, 2) = 2 * (c * d - a * b);
            Rotm.At<double>(2, 0) = 2 * (b * d - a * c);
            Rotm.At<double>(2, 1) = 2 * (c * d + a * b);
            Rotm.At<double>(2, 2) = 2 * (a * a + d * d) - 1;
            return Rotm;
        }


        public static Mat ConstructPoseMatrix(Mat Rot, Matrix<double> Trans, int num_M) // construct transformation matrix, combine rot, trans matrix
        {
            int n_row = Rot.Rows;
            int n_col = Rot.Cols;
            Mat Bpre = new Mat(4, 4, MatType.CV_64F);
            double[] temp = { 0, 0, 0, 1 };


            for (int i = 0; i < n_row; i++)
            {
                for (int j = 0; j < n_col; j++)
                {
                    Bpre.At<double>(i, j) = Rot.At<double>(i, j);
                }
            }

            for (int i = 0; i < n_row; i++)
            {

                Bpre.At<double>(i, 3) = Trans[num_M, i];
            }

            for (int j = 0; j < 4; j++)
            {
                Bpre.At<double>(3, j) = temp[j];
            }

            //printMat(Bpre);

            return Bpre;
        }
    }
}
