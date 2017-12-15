/* ----------------------------------------------------------------------------
 * Robotics Laboratory, Westphalian University of Applied Science
 * ----------------------------------------------------------------------------
 * Project			: 	Stereo Vision Project
 * Revision			: 	1.0
 * Recent changes	: 	18.06.2014	 
 * ----------------------------------------------------------------------------
 * LOG:
 * ----------------------------------------------------------------------------
 * Developer		: 	Dennis Luensch 		(dennis.luensch@gmail.com)
						Tom Marvin Liebelt	(fh@tom-liebelt.de)
						Christian Blesing	(christian.blesing@gmail.com)
 * License 			: 	BSD 
 *
 * Copyright (c) 2014, Dennis Lünsch, Tom Marvin Liebelt, Christian Blesing
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * # Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * # Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * # Neither the name of the {organization} nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ------------------------------------------------------------------------- */

#include <iostream>
#include "stereoprocessor.h"
#include "imageprocessor.h"
#include <cstdlib>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    bool readSuccessfully = false;
    bool success = false;

    uint dMin; uint dMax; Size censusWin; float defaultBorderCost;
    float lambdaAD; float lambdaCensus; string savePath; uint aggregatingIterations;
    uint colorThreshold1; uint colorThreshold2; uint maxLength1; uint maxLength2; uint colorDifference;
    float pi1; float pi2; uint dispTolerance; uint votingThreshold; float votingRatioThreshold;
    uint maxSearchDepth; uint blurKernelSize; uint cannyThreshold1; uint cannyThreshold2; uint cannyKernelSize;

    dMin = 0;
    dMax = 60;
    censusWin.height = 9;
    censusWin.width = 7;
    defaultBorderCost = 0.999;
    lambdaAD = 10.0; 
    lambdaCensus = 30.0;
    savePath = "../results/";
    aggregatingIterations = 4;
    colorThreshold1 = 20;
    colorThreshold2 = 6;
    maxLength1 = 34;
    maxLength2 = 17;
    colorDifference = 15;
    pi1 = 0.1;
    pi2 = 0.3;
    dispTolerance = 0;
    votingThreshold = 20;
    votingRatioThreshold = 0.4;
    maxSearchDepth = 20;
    cannyThreshold1 = 20;
    cannyThreshold2 = 60;
    cannyKernelSize = 3;
    blurKernelSize = 3;

    bool error = false;
    int frame_num = atoi(argv[1]);
    for (int i = 0; i < frame_num && !error; ++i)
    {
        stringstream file;
        file << savePath << i;
        
        std::clock_t start = std::clock();

        ImageProcessor iP(0.1);
        Mat eLeft, eRight, imleft, imright;
        imleft = imread(argv[2], CV_LOAD_IMAGE_UNCHANGED);
        imright = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
        if (imleft.rows > 0 && imright.rows > 0 )
        {
            readSuccessfully = true;
        }
        else
        {
            std::cout << "Image " << i << "cannot be loaded" << std::endl;
            continue; 
        }
        eLeft = iP.unsharpMasking(imleft, "gauss", 3, 1.9, -1);
        eRight = iP.unsharpMasking(imright, "gauss", 3, 1.9, -1);

        StereoProcessor sP(dMin, dMax, imleft, imright, censusWin, defaultBorderCost, lambdaAD, lambdaCensus, file.str(),
                            aggregatingIterations, colorThreshold1, colorThreshold2, maxLength1, maxLength2,
                            colorDifference, pi1, pi2, dispTolerance, votingThreshold, votingRatioThreshold,
                            maxSearchDepth, blurKernelSize, cannyThreshold1, cannyThreshold2, cannyKernelSize);
        string errorMsg;
        error = !sP.init(errorMsg);

        if(!error && sP.compute())
        {
            success = true;
            Mat disp = sP.getDisparity();
        }
        else
        {
            cerr << "[ADCensusCV] " << errorMsg << endl;
        }
        std::clock_t end = std::clock();
        double diff = (end - start) / (double)CLOCKS_PER_SEC;;
        cout << "Computation time: " << diff << "s" << endl;
    }
}

