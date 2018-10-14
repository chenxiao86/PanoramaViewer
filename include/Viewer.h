#ifndef VIEWER_H
#define VIEWER_H

#include<string>
#include<vector>
#include<list>

using namespace std;
struct ImagePoint
{
    float x, y, z;
    float r, g, b;
    float alpha;
};
class MapDrawer;
class Viewer
{
public:
    Viewer(vector<string> vPaths);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void DrawPoints(float axis[3]);
    ImagePoint toUnitSphere(int idx, unsigned char r, unsigned char g, unsigned char b);

    void ReadImage(bool next);

private:

    vector<string> mvPaths;
    int mnImgIdx;
    float mPtSize;
    int mRows, mCols;
    int N;
    float F;
    bool mbNew;

    vector<ImagePoint> mvPoints;

    // 1/fps in ms
    double mT;
};



#endif // VIEWER_H
	

