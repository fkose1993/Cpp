#include "stdafx.h"
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <string> 
#include <fstream>


using namespace cv;
using namespace std;

int morph_elem = 0;
int morph_size = 1; //2
vector<Point> intersectionPoint(Mat, Point, Point, Point, Point);
double uzunluk_hesapla(Point, Point);
double egim_hesapla(Point, Point);
double aci_hesapla(double);
double min_uzaklik = 9999;

#define PI 3.14159265

int basilan;
int uzaklik;
Point tmp;
Vec4i l1, l2;
vector<Point> hull;
vector<vector<Point>> noktalar;
vector<Vec4i> lines, lines1;
vector<Point> kesisim_noktalari, temp;
vector<Point> koseler;
Point2f vertices[4];
Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
RotatedRect cevreleyen_dortgen;

Mat dst, cdst, son, bsrc;

String filename, point;
Size boyut;
Mat src, resim;
RNG rng(12345);
ofstream myFile;


int alt_limit = 1, ust_limit = 40, durum = alt_limit;

int main(int argc, char** argv)
{

//Text file that contains coordinates of 4 corners.
	myFile.open("C:\\Users\\fatihköse\\Downloads\\kup1\\kirliOrtam1.txt", ios_base::app);


	if (!myFile.is_open())
	{
		cout << "Unable to open file";
		return 0;
	}




	while (durum >= alt_limit && durum <= ust_limit)
	{


		//Clean vectors
		noktalar.clear();
		hull.clear();
		lines.clear();
		kesisim_noktalari.clear();
		koseler.clear();


		system("cls");


		//Path of image
		filename = "C:\\Users\\fatihköse\\Downloads\\kup1\\kirliOrtam (";
		filename = filename + to_string(durum);
		filename += ").jpg";
		point = filename;

		//Get image
		src = imread(filename, 1);
		resim = src;
		cvtColor(src, src, CV_BGR2GRAY);

		//Resize image
		boyut = src.size();
		resize(src, src, Size(boyut.width / 10, boyut.height / 10));
		resize(resim, resim, Size(boyut.width / 10, boyut.height / 10));

		if (src.empty())
		{
			cout << "can not open " << filename << endl;
			return 0;
		}


		//Filter to reduce noise.
		bilateralFilter(src, bsrc, 3, 150, 150);

		//Canny Edge Detection
		Canny(bsrc, dst, 44, 110, 3);
		cvtColor(dst, cdst, CV_GRAY2BGR);


		//Finding straight lines (Hough transform)
		HoughLinesP(dst, lines1, 1, CV_PI / 180, 50, 75, 10);

		//Draw lines
		for (size_t i = 0; i < lines1.size(); i++)
		{
			l1 = lines1[i];
			line(dst, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(255, 255, 255), 2, CV_AA);
			line(cdst, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(255, 255, 255), 2, CV_AA);
		}



		// Morph Closing (Fill gap between close lines)
		morphologyEx(dst, dst, MORPH_CLOSE, element);

		//Convert image to binary for edge detecting
		threshold(dst, dst, 0, 255, 0);



		//Find straight lines again
		HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 75, 10); // threshold 50 , 65

		//Draw lines
		for (size_t i = 0; i < lines.size() - 1; i++)
		{
			l1 = lines[i];
			line(cdst, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(255, 255, 255), 2, CV_AA);
			line(resim, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(0, 0, 255), 2, CV_AA);
			for (size_t j = i + 1; j < lines.size(); j++) // Check if lines are intersecting vertically
			{
				l2 = lines[j];
				temp = intersectionPoint(bsrc, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Point(l2[0], l2[1]), Point(l2[2], l2[3]));

				for (int k = 0; k < temp.size(); k++)
				{
					kesisim_noktalari.push_back(temp[k]);// Push start-end-intersecting points of intersecting lines.
				}
				temp.clear();
			}


		}

		cout << filename << endl << endl;

		int yakin_nokta_sayisi, tempx = 0, tempy = 0;

		for (int i_count = 0; i_count < kesisim_noktalari.size(); i_count++) // Start-end-intersecting points are drawn as little circles.
		{
			yakin_nokta_sayisi = 0;
			circle(cdst, kesisim_noktalari[i_count], 1, Scalar(255, 255, 255), 3, 8, 0);
			circle(resim, kesisim_noktalari[i_count], 1, Scalar(255, 255, 255), 3, 8, 0);

			tempx += kesisim_noktalari[i_count].x;
			tempy += kesisim_noktalari[i_count].y;
      
			// To reduce error that caused by lines found outside the rubik cube. (Not optimised)
      
      /*	for (int i_count2 = i_count + 1; i_count2 < kesisim_noktalari.size(); i_count2++)
			{ // To reduce error that caused by lines found outside the rubik cube. (Not optimised)
			if (uzunluk_hesapla(kesisim_noktalari[i_count], kesisim_noktalari[i_count2]) <= 75)
			yakin_nokta_sayisi++;
			}

			if (yakin_nokta_sayisi < 2)
			kesisim_noktalari.erase(kesisim_noktalari.begin() + i_count, kesisim_noktalari.begin() + i_count+1);
			*/
		}


		if (kesisim_noktalari.size()>0)
		{
			tempx /= kesisim_noktalari.size();
			tempy /= kesisim_noktalari.size();
		}



		try{
			convexHull(kesisim_noktalari, hull, false); // Calculate smallest contour that covers all start-end-intersection points.
			noktalar.push_back(hull);
			drawContours(src, noktalar, 0, Scalar(255, 255, 255), 2, 8, vector<Point>()); // Draw contour.


			// Calculate smallest rectangle that covers the contour.

			cevreleyen_dortgen = minAreaRect(noktalar[0]);
			cevreleyen_dortgen.points(vertices);



			// Number rubik cube's corners. (By distance from corners of image.)

			for (int l = 0; l < 4; l++)
			{
				uzaklik = uzunluk_hesapla(vertices[l], Point(0, 0));

				if (uzaklik <= min_uzaklik)
				{
					min_uzaklik = uzaklik;
					tmp = vertices[l];
				}

			}

			koseler.push_back(tmp);
			min_uzaklik = 9999;

			for (int l2 = 0; l2 < 4; l2++)
			{
				uzaklik = uzunluk_hesapla(vertices[l2], Point(0, src.rows - 1));

				if (uzaklik <= min_uzaklik)
				{
					min_uzaklik = uzaklik;
					tmp = vertices[l2];
				}

			}
			koseler.push_back(tmp);
			min_uzaklik = 9999;




			for (int l3 = 0; l3 < 4; l3++)
			{
				uzaklik = uzunluk_hesapla(vertices[l3], Point(src.cols - 1, src.rows - 1));

				if (uzaklik <= min_uzaklik)
				{
					min_uzaklik = uzaklik;
					tmp = vertices[l3];
				}

			}
			koseler.push_back(tmp);
			min_uzaklik = 9999;


			for (int l4 = 0; l4 < 4; l4++)
			{
				uzaklik = uzunluk_hesapla(vertices[l4], Point(src.cols - 1, 0));

				if (uzaklik <= min_uzaklik)
				{
					min_uzaklik = uzaklik;
					tmp = vertices[l4];
				}

			}
			koseler.push_back(tmp);
			min_uzaklik = 9999;


			for (int i = 0; i < 4; i++) //Draw rectange.
			{
				line(resim, koseler[i], koseler[(i + 1) % 4], cv::Scalar(0, 255, 0), 3, CV_AA);
				putText(resim, to_string(i), koseler[i], FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0), 2);
				//   C:\\PATH\\PATH||point1:(203,105)||point2:(201,232)||point3:(329,231)||point4:(329,105) <<Example format
        // to record corner points that algorithm found to a file. (For comparing with real coordinates that determined by human.)
				point += "||";
				point += to_string((int)koseler[i].x);
				point += ",";
				point += to_string((int)koseler[i].y);
			}


			putText(resim, "X", Point(tempx, tempy), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0), 2); // Center of rubik cube's face

		}
		catch (Exception e) //Error message (Contour or rectange cannot calculated.)
		{
			cout << "Contour veya dortgen hesaplanamadi !";
		}


		/*

		Here we can check if rectangle close to a square.

		*/





		point += ";";

		imshow("source", src);
		imshow("detected lines", cdst);
		imshow("renkli", resim);




		//Change image by pressing buttons (X= next , Z= previous)
		
		basilan = waitKey(0);

		if (basilan == 27)
		break;

		if (basilan == 'z' || basilan == 'Z')
		{
		durum--;
		}

		if (basilan == 'x' || basilan == 'X')
		{
		durum++;
		}
		


		//Automaticly change image
		/*
		basilan = waitKey(1000);

		if (basilan == 27)
			break;

		durum++;
		*/

		myFile << point << endl;

	}

	myFile.close();
	return 0;
}



vector<Point> intersectionPoint(Mat resim, Point a1, Point a2, Point b1, Point b2)
{ // Function that checks if 2 lines intersecting.


	LineIterator line1(resim, a1, a2, 8);
	LineIterator line2(resim, b1, b2, 8);


	double m1, m2, line_aci_farki = -5;

	m1 = egim_hesapla(a1, a2);
	m2 = egim_hesapla(b1, b2);


	if (m1*m2 != -1)
	{
		line_aci_farki = aci_hesapla(abs((m2 - m1) / (1 + m1*m2)));
	}

	else
	{
		line_aci_farki = 90;
	}

	vector<Point> l1, l2, l3;


	for (int i = 0; i < line1.count; i++, ++line1)
		l1.push_back(line1.pos());

	for (int k = 0; k < line2.count; k++, ++line2)
		l2.push_back(line2.pos());


	for (int j = 0; j < l1.size(); j++)
		for (int m = 0; m < l2.size(); m++)
			if (uzunluk_hesapla(l1[j], l2[m]) <= 1 && line_aci_farki >= 45)
			{

				m1 = egim_hesapla(l2[m], a2);
				m2 = egim_hesapla(l2[m], b2);


				if (m1*m2 != -1)
				{
					line_aci_farki = aci_hesapla(abs((m2 - m1) / (1 + m1*m2)));
				}

				else
				{
					line_aci_farki = 90;
				}


				if (line_aci_farki >= 80 && line_aci_farki <= 90)
				{
					l3.push_back(l1[j]); // intersecting point
					l3.push_back(l2[m]);
					//start-end points of 2 lines that are intersecting
					l3.push_back(a1);
					l3.push_back(a2);
					l3.push_back(b1);
					l3.push_back(b2);
				}
				line_aci_farki = -5;
			}

	return l3;

}


//Function calculates distance between two points.
double uzunluk_hesapla(Point p1, Point p2)
{
	return (sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2)));

}


//Function calculates slope of a line.
double egim_hesapla(Point p1, Point p2)
{
	if (p2.x - p1.x != 0)
		return (p2.y - p1.y) / (p2.x - p1.x);

	else if (p2.y - p1.y < 0)
		return -200;
	else
		return 200;
}


// Function calculates angle from slope.
double aci_hesapla(double e)
{
	return atan(e) * 180 / PI;

}
