#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//vector<Point2f>contours;
vector<vector<Point> > contours2;
int findBiggestContour(vector<vector<Point>>contour)
{
    int indexOfBiggestContour = -1;
    int sizeOfBiggestContour = 0;
    for (int i=0;i<contour.size();i++) {
        if (contour[i].size() > sizeOfBiggestContour){
            sizeOfBiggestContour=contour[i].size();
            indexOfBiggestContour=i;
        }
    }
    return indexOfBiggestContour;
}
void Mor(Mat mask2)
{
    Mat element = getStructuringElement(MORPH_CROSS,Size(1,5));
    morphologyEx(mask2,mask2,MORPH_CROSS,element,Point(-1,-1),8);
    dilate( mask2, mask2, element );
    dilate( mask2, mask2, element );
    dilate( mask2, mask2, element );
    erode( mask2, mask2,element );
    erode( mask2, mask2, element );
    erode( mask2, mask2, element );
    //    erode( mask2, mask2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //        morphologyEx(mask2,mask2,MORPH_CLOSE,element,Point(-1,-1),3);
    //    morphologyEx(mask2,mask2,MORPH_OPEN,element,Point(-1,-1),16);
}
Mat pyrup(Mat Roi)
{
    pyrUp(Roi,Roi);
    pyrUp(Roi,Roi);
    pyrUp(Roi,Roi);
    return Roi;
}
Mat pyrdown(Mat mask2)
{
    Mat prup2;
    pyrDown(mask2,prup2);
    pyrDown(prup2,prup2);
    pyrDown(prup2,prup2);
    return  prup2;
}
int main()
{
    VideoCapture cam(1);
    Point center(0,0);
    Mat sample = imread("C:/Users/ksrnd/Desktop/asfasdgasgeg_a.png");
    namedWindow("Control",CV_WINDOW_AUTOSIZE);

    int match_method=0;
    createTrackbar("Method", "Control", &match_method ,5);
    while (true) {
        Mat frame;
        cam >> frame;

        int result_cols = frame.cols - sample.cols +1;
        int result_rows = frame.rows - sample.rows +1;

        Mat result;
        result.create(result_rows,result_cols,CV_32FC1);

        matchTemplate(frame,sample,result,match_method);
        normalize(result,result,0,1,NORM_MINMAX,-1,Mat());

        double minVal; double maxVal; Point minLoc; Point maxLoc;
        Point matchLoc;
        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

        if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
        else
        { matchLoc = maxLoc; }

        //        rectangle( frame, Point( matchLoc.x - 20 , matchLoc.y - 20 ), Point( matchLoc.x + sample.cols +20 , matchLoc.y + sample.rows+20 ), Scalar::all(0), 2, 8, 0 );
        //        rectangle( result, matchLoc, Point( matchLoc.x + sample.cols , matchLoc.y + sample.rows ), Scalar::all(0), 2, 8, 0 );

        Mat Roi;

        Roi= frame(Rect(Point( matchLoc.x - 20 , matchLoc.y - 20 ) ,Point( matchLoc.x + sample.cols +20, matchLoc.y + sample.rows +20 )));

        Mat newRoi = pyrup(Roi);

        //speed problem
        Mat gray,hsv,hsv2;
        cvtColor(newRoi, hsv, COLOR_BGR2HSV);

        Mat mask,mask2;
        inRange(hsv, Scalar(0, 130, 0), Scalar(179, 255, 255), mask);
        Mat mask_inv;
        bitwise_not(mask,mask_inv,noArray());

        Mat Test,Test2;
        bitwise_and(newRoi,newRoi,Test,mask_inv);

        Mat crane;
        Test.copyTo(crane);
        cvtColor(crane, hsv2, COLOR_BGR2HSV);
        inRange(hsv2, Scalar(71, 35, 179), Scalar(179, 255, 255), mask2);
        Mor(mask2);
        Mat PryDownRoi;
        PryDownRoi=pyrdown(mask2);
        imshow("hsv",PryDownRoi);



        findContours( PryDownRoi, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        int BiggestContours=findBiggestContour(contours2);
        ///DEFINE: Using rectangle for Rect
        //        vector<vector<Point> > contours_poly( contours2.size() );
        //        vector<vector<Point> > hull( contours2.size() );
        //        vector<Rect> boundRect( contours2.size() );
        //        approxPolyDP( Mat(contours2[BiggestContours]), contours_poly[BiggestContours], 3, true );
        //        boundRect[BiggestContours] = boundingRect( Mat(contours_poly[BiggestContours]) );
        //        rectangle( Roi, boundRect[BiggestContours].tl(), boundRect[BiggestContours].br(), Scalar(0,0,255), 2, 8, 0 );
        //        circle(Roi,Point(boundRect[BiggestContours].x,boundRect[BiggestContours].y+boundRect[BiggestContours].height),5,Scalar(255,0,0),3);
        //        circle(Roi,Point(boundRect[BiggestContours].x+boundRect[BiggestContours].width,boundRect[BiggestContours].y),5,Scalar(255,0,0),3);
        //        convexHull( Mat(contours2[BiggestContours]), hull[BiggestContours], false);
        //        drawContours( Roi, contours_poly, BiggestContours, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );

        vector<RotatedRect> minRect(contours2.size());
        vector<RotatedRect> minEllipse(contours2.size());
        minRect[BiggestContours] = minAreaRect(Mat(contours2[BiggestContours]));
        minEllipse[BiggestContours] = fitEllipse( Mat(contours2[BiggestContours]) );
        Point2f rect_points[4];
        minRect[BiggestContours].points( rect_points );
        for( int j = 0; j < 4; j++ ){
            line( Roi, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 1, 8 );
            circle(Roi,Point((rect_points[0].x+rect_points[1].x)/2,(rect_points[0].y+rect_points[1].y)/2),5,Scalar(255,0,0),5);
            circle(Roi,Point((rect_points[2].x+rect_points[3].x)/2,(rect_points[2].y+rect_points[3].y)/2),5,Scalar(255,0,0),5);

        }
//        imshow("ROi",Roi);
        imshow("frame",frame);

        if (waitKey(1)==27)break;
    }
}
