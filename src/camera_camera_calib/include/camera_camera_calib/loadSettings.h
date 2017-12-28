#ifndef LOADSETTINGS_H
#define LOADSETTINGS_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, APRILTAG, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Column"  << boardSize.width
                  << "BoardSize_Row" << boardSize.height
                  << "Image_Width" << imageWidth
                  << "Image_Height" << imageHeight
                  << "Calibrate_Pattern" << patternToUse
                  << "InitialRotation" << initialRotation
                  << "InitialTranslation" << initialTranslation;  
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Column" ] >> boardSize.width;
        node["BoardSize_Row"]     >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Image_Width"] >> imageWidth;
        node["Image_Height"] >> imageHeight;
        node["InitialRotation"] >> initialRotation;
        node["InitialTranslation"] >> initialTranslation;

        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        // if (squareSize <= 10e-6)
        // {
        //     cerr << "Invalid square size " << squareSize << endl;
        //     goodInput = false;
        // }


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("APRILTAG")) calibrationPattern = APRILTAG;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], CV_LOAD_IMAGE_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }

    static bool isListOfImages( const string& filename)
    {
        string s(filename);
        // Look for file extension
        if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
            return false;
        else
            return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern

    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->

    
    // Size resolution;            // Camera resolution 
    int imageWidth;
    int imageHeight;
    Mat initialRotation;
    Mat initialTranslation;

    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;; // unit: %


private:
    string patternToUse;


};


class AprilTagOcamConfig : public Settings       // make AprilTag OcamCalib derive from Setting parent
{
public:

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        Settings::write(fs);

        fs  << "Tag_Size"  << tagSize
            << "Tag_Space" << tagSpace
            << "}";
    }

    void read(const FileNode& node)           //Read serialization for this class
    {   
        node["Tag_Size"]  >> tagSize;
        node["Tag_Space"] >> tagSpace;
        Settings::read(node);
    }

public:
    double tagSize;    // in millimeter
    double tagSpace;
};

class AprilTagOmniConfig : public Settings       // make AprilTag OcamCalib derive from Setting parent
{
public:

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        Settings::write(fs);

        fs  << "Cam0Intrinsics" << intrinsics0
            << "Cam0Distortion" << distortion0
            << "Cam0MirrorPara" << xi0
            << "Cam1Intrinsics" << intrinsics1
            << "Cam1Distortion" << distortion1
            << "Cam1MirrorPara" << xi1
            << "Tag_Size"       << tagSize
            << "Tag_Space" << tagSpace
            << "}";
    }

    void read(const FileNode& node)           //Read serialization for this class
    {
        node["Cam0Intrinsics"] >> intrinsics0;
        node["Cam0Distortion"] >> distortion0;
        node["Cam0MirrorPara"] >> xi0;
        node["Cam1Intrinsics"] >> intrinsics1;
        node["Cam1Distortion"] >> distortion1;
        node["Cam1MirrorPara"] >> xi1;
        node["Tag_Size"]  >> tagSize;
        node["Tag_Space"] >> tagSpace;
        Settings::read(node);
    }

public:
    Mat intrinsics0;   // Camera intrinsic paramter
    Mat distortion0;   // Camera distortion coefficients
    double xi0;        // Mirror parameter

    Mat intrinsics1;   // Camera intrinsic paramter
    Mat distortion1;   // Camera distortion coefficients
    double xi1;        // Mirror parameter

    double tagSize;    // in millimeter
    double tagSpace;   

};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

static void read(const FileNode& node, AprilTagOcamConfig& x, const Settings& default_value = Settings())
{
    
    x.read(node);
}

static void read(const FileNode& node, AprilTagOmniConfig& x, const Settings& default_value = Settings())
{
    
    x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );
#endif