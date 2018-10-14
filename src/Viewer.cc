/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<chrono>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <iostream>

#define PI 3.14159265358979f

using namespace cv;

namespace pangolin {
struct RotHandler3D : Handler3D
{
    RotHandler3D(OpenGlRenderState &cam_state, AxisDirection enforce_up = AxisNone,
                 float trans_scale = 0.01f, float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF)
        : Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction) {
        F = 2000;
    }
    int funcKeyState;

    void Keyboard(View&, unsigned char key, int x, int y, bool pressed)
    {
        *ptSize = (key=='+'?1.05:1)*(*ptSize);
        *ptSize = (key=='-'?0.95:1)*(*ptSize);
        if((double)key == 230 && *mbNext == false && pressed) *mbNext = true;
        if((double)key == 228 && *mbPrev == false && pressed) *mbPrev = true;
    }
    
    void Mouse(View &display, MouseButton button, int x, int y,
               bool pressed, int button_state)
    {
        // mouse down
        last_pos[0] = (float)x;
        last_pos[1] = (float)y;
        
        GLprecision T_nc[3*4];
        LieSetIdentity(T_nc);
        
        funcKeyState = 0;
        if( pressed )
        {
            GetPosNormal(display,x,y,p,Pw,Pc,n,last_z);
            if( ValidWinDepth(p[2]) )
            {
                last_z = p[2];
                std::copy(Pc,Pc+3,rot_center);
            }
            
            if( button == MouseWheelUp || button == MouseWheelDown)
            {
                F = (button==MouseWheelUp?1.05:0.95)*F;
                *ptSize = (button==MouseWheelUp?1.05:0.95)*(*ptSize);

                cam_state->SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,F,F,512,389,0.1,1000));
            }
            
            funcKeyState = button_state;
        }
    }
    void MouseMotion(View& display, int x, int y, int button_state)
    {
        const GLprecision rf = 0.01;
        const float delta[2] = { (float)x - last_pos[0], (float)y - last_pos[1] };
        const float mag = delta[0]*delta[0] + delta[1]*delta[1];
        
        if((button_state & KeyModifierCtrl) && (button_state & KeyModifierShift))
        {
            GLprecision T_nc[3 * 4];
            LieSetIdentity(T_nc);
            
            GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
            if(ValidWinDepth(p[2]))
            {
                last_z = p[2];
                std::copy(Pc, Pc + 3, rot_center);
            }
            
            funcKeyState = button_state;
        }
        else
        {
            funcKeyState = 0;
        }
        
        if( mag < 50.0f*50.0f )
        {
            OpenGlMatrix& mv = cam_state->GetModelViewMatrix();
            const GLprecision* up = AxisDirectionVector[enforce_up];
            GLprecision T_nc[3*4];
            LieSetIdentity(T_nc);
            bool rotation_changed = false;
            
            if( button_state == MouseButtonLeft )
            {
                // Try to correct for different coordinate conventions.
                GLprecision aboutx = -rf * delta[1];
                GLprecision abouty = rf * delta[0];
                OpenGlMatrix& pm = cam_state->GetProjectionMatrix();
                abouty *= -pm.m[2 * 4 + 3];
                
                Rotation<>(T_nc, aboutx, abouty, (GLprecision)0.0);
            }
            else if( button_state == MouseButtonRight )
            {
                GLprecision T_2c[3*4];
                Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, delta[0]*rf);
                GLprecision mrotc[3];
                MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
                LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
                GLprecision T_n2[3*4];
                LieSetIdentity<>(T_n2);
                LieSetTranslation<>(T_n2,rot_center);
                LieMulSE3(T_nc, T_n2, T_2c );
                rotation_changed = true;
            }
            
            LieMul4x4bySE3<>(mv.m,T_nc,mv.m);
            mv.m[12] = 0; mv.m[13] = 0; mv.m[14] = 0;
            
            if(enforce_up != AxisNone && rotation_changed) {
                EnforceUpT_cw(mv.m, up);
            }
        }
        
        last_pos[0] = (float)x;
        last_pos[1] = (float)y;
    }
    
    
    float F;
    float* ptSize;
    bool *mbPrev, *mbNext;
    
};
}

Viewer::Viewer(vector<string> vPaths): mvPaths(vPaths), mnImgIdx(-1), mbNew(true)
{
    float fps = 30;
    mT = 1e3/fps;
    F = 2000;
    
    N = mvPaths.size();
    mPtSize = 5;
    ReadImage(true);
}

void Viewer::Run()
{
    
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
    
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    
    pangolin::Var<bool> menuPrevImg("menu.PrevImg",false,false);
    pangolin::Var<bool> menuNextImg("menu.NextImg",false,false);
    bool bNextImg = false, bPrevImg = false;
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,F,F,512,384,0.1,1000),
                pangolin::ModelViewLookAt(0,0,0, 0,0,1,0.0,-1.0, 0.0)
                );

    pangolin::RotHandler3D *mouseHandler = new pangolin::RotHandler3D(s_cam);
    mouseHandler->F = F;
    mouseHandler->ptSize = &mPtSize;
    mouseHandler->mbNext = &bNextImg;
    mouseHandler->mbPrev = &bPrevImg;
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(mouseHandler);
    
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        pangolin::OpenGlMatrix TT = s_cam.GetModelViewMatrix();
        float axis[3] = {TT.m[2],TT.m[6],TT.m[10]};\
        
        if(menuReset)
        {
            Twc.SetIdentity();
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,0, 0,0,1,0.0,-1.0, 0.0));
            mouseHandler->F = F;
            s_cam.Follow(Twc);
            menuReset = false;
        }
        if(menuNextImg || bNextImg)
        {
            ReadImage(true);
            menuNextImg = false;
            bNextImg = false;
        }
        if(menuPrevImg || bPrevImg)
        {
            ReadImage(false);
            menuPrevImg = false;
            bPrevImg = false;
        }
        s_cam.Follow(Twc);
        
        //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        
        DrawPoints(axis);
        //std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        pangolin::FinishFrame();
        //std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        
        //double time1 = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        //double time2 = std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
        //double time3 = std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
        //cout<<"Time active: "<<time1<<", time set points: "<<time2<<", time finish frame: "<<time3<<endl;
        
        cv::waitKey(mT);
    }
    
}
void Viewer::ReadImage(bool next)
{
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    bool foundImage = false;
    cv::Mat img;
    for(int i=0; i<N; i++)
    {
        if(next)
            mnImgIdx++;
        else
            mnImgIdx--;
        if(mnImgIdx<0)
            mnImgIdx += N;
        else if(mnImgIdx >= N)
            mnImgIdx -= N;
        
        img = cv::imread(mvPaths[mnImgIdx]);
        if(img.empty()) continue;
        if(img.cols> 3000)
            cv::resize(img, img, cv::Size(), 0.5, 0.5);
        
        /// read image
        mRows = img.rows;
        mCols = img.cols;
        mvPoints.clear();
        mvPoints.resize(mRows*mCols);
        int idx = 0;
        if(img.type() == CV_8UC1)
        {
            cv::MatIterator_<uchar> iter = img.begin<uchar>();
            cv::MatIterator_<uchar> iterEnd = img.end<uchar>();
            for(; iter != iterEnd; iter++, idx++)
            {
                uchar I = *iter;
                mvPoints[idx] = toUnitSphere(idx, I, I, I);
            }
        }
        else if(img.type() == CV_8UC3)
        {
            cv::MatIterator_<cv::Vec3b> iter = img.begin<cv::Vec3b>();
            cv::MatIterator_<cv::Vec3b> iterEnd = img.end<cv::Vec3b>();
            for(; iter != iterEnd; iter++, idx++)
            {
                mvPoints[idx] = toUnitSphere(idx, (*iter)[2], (*iter)[1], (*iter)[0]);
            }
        }
        else
        {
            cerr<< "Image is not in 1 or 3 channels??"<<endl;
            continue;
        }
        
        foundImage = true;
        cout<<"Show image"<< mvPaths[mnImgIdx] <<endl;
        break;
    }
    
    if(!foundImage)
    {
        cerr<< "No image found!"<<endl;
        exit(-1);
    }
}
ImagePoint Viewer::toUnitSphere(int idx, unsigned char r, unsigned char g, unsigned char b)
{
    ImagePoint ret;
    ret.r = r/255.0f;
    ret.g = g/255.0f;
    ret.b = b/255.0f;
    
    float y = floor(idx/mCols);
    float x = idx - y*mCols;
    
    x = x/mCols*2*PI;
    y = y/mRows*PI;
    
    ret.x = sin(x)*sin(y);
    ret.y = -cos(y);
    ret.z = cos(x)*sin(y);
    
    return ret;
}

void Viewer::DrawPoints(float axis[3])
{
    
    glPointSize(mPtSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    
    for(vector<ImagePoint>::iterator iter=mvPoints.begin(),
        iend=mvPoints.end(); iter!=iend;iter++)
    {
        ImagePoint &pt = *iter;
        if(pt.x*axis[0] + pt.y*axis[1] + pt.z*axis[2] > -0.1) continue;
        
        glColor3f(pt.r, pt.g, pt.b);
        glVertex3f(pt.x, pt.y, pt.z);
        glNormal3f(-pt.x, -pt.y, -pt.z);
    }
    glEnd();
}
