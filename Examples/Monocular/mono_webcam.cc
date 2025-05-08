#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_live path_to_vocabulary path_to_settings [trajectory_file_name]" << endl;
        return 1;
    }

    string vocab_path = argv[1];
    string settings_path = argv[2];

    bool save_traj = (argc == 4);
    string traj_filename;
    if(save_traj)
        traj_filename = argv[3];

    // ORB-SLAM3 sistemini başlat
    ORB_SLAM3::System SLAM(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Kamerayı başlat
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
    {
        cerr << "Kamera açılamadı!" << endl;
        return -1;
    }

    cout << "Gerçek zamanlı ORB-SLAM3 çalışıyor. Çıkmak için ESC'ye basın." << endl;

    // Takip süresi istatistikleri
    vector<float> vTimesTrack;
    int frame_id = 0;

    while(true)
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;

        if(imageScale != 1.f)
        {
            int width = frame.cols * imageScale;
            int height = frame.rows * imageScale;
            cv::resize(frame, frame, cv::Size(width, height));
        }

        // Zaman etiketi oluştur
        double tframe = chrono::duration_cast<chrono::duration<double>>(
                            chrono::steady_clock::now().time_since_epoch()).count();

        auto t1 = chrono::steady_clock::now();
        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame, tframe);
        auto t2 = chrono::steady_clock::now();

        float ttrack = chrono::duration_cast<chrono::duration<float>>(t2 - t1).count();
        vTimesTrack.push_back(ttrack);

        // Pozisyon bilgisi terminale yazdır
        if (!Tcw.translation().isZero())
        {
            auto pos = Tcw.translation(); // Eigen::Vector3f
            cout << fixed << setprecision(6);
            cout << "Frame " << frame_id << " | Pozisyon [x y z]: "
                 << pos.x() << " "
                 << pos.y() << " "
                 << pos.z() << endl;
        }

        cv::imshow("ORB-SLAM3 Live", frame);
        if(cv::waitKey(1) == 27) break; // ESC tuşu ile çık

        frame_id++;
    }

    // SLAM'i durdur
    SLAM.Shutdown();

    // Trajectory kaydet (isteğe bağlı)
    if(save_traj)
    {
        SLAM.SaveTrajectoryEuRoC(traj_filename + "_trajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC(traj_filename + "_keyframes.txt");
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}
