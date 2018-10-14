#include"Viewer.h"
#include <unistd.h>
#include <dirent.h>
#include<iostream>
#include<algorithm>

void LoadImages(string strImagePath, vector<string> &vFileNames)
{
    DIR *dp;
    struct dirent *dirp;
    if(strImagePath[strImagePath.size()-1] != '/')
        strImagePath = strImagePath+"/";

    if((dp  = opendir(strImagePath.c_str())) == NULL)
        return;

    while ((dirp = readdir(dp)) != NULL) {
        string name = string(dirp->d_name);
        if(name.size()<4) continue;
        string suf = name.substr(name.size()-3,3);

        if( 1 || suf == string("jpg") || suf == string("JPG")
              || suf == string("png") || suf == string("PNG")
              || suf == string("bmp") || suf == string("BMP")
              || suf == string("jpeg") || suf == string("JPEG"))
        {
            vFileNames.push_back(strImagePath+name);
            cout<<"Found file: "<<name<<endl;
        }

    }
    closedir(dp);

    std::sort(vFileNames.begin(), vFileNames.end());

    // Output loading result
    cout << vFileNames.size() << " images were loaded" << endl;
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        cerr<< "Usage: ./main {Path of Images}" <<endl;
        return -1;
    }
    vector<string> vPaths;
    string strImagePath(argv[1]);
    LoadImages(strImagePath, vPaths);

    cout << "Left and right button: rotation; "<<endl
         << "+/-: change point size;"<<endl
         << "<-/->: previous or next image."<<endl;

    Viewer *pViewer = new Viewer(vPaths);
    pViewer->Run();
}

