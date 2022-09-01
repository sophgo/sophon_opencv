#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/freetype.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {	
    try {
        Mat img;
		const String msg = "汉字 iw0ol1 ニホンゴ";
		const String msg2 = "조선말／朝鮮말，한국어／韓國語";

		Ptr<freetype::FreeType2> ft2;
		ft2 = freetype::createFreeType2();
		
		if (argc < 3){
			cout << argv[0] << " pic_in pic_out" << endl;
			return -1;
		}
		
		img = imread(argv[1]);
		
        ft2->loadFontData( "/data/wqy-microhei.ttc", 0 );
		cout << "Load fonts successfully." << endl;
		
		int baseline=0;
		int fontHeight = 60;
		int thickness = -1;
		int linestyle = 8;
		
		Size textSize = ft2->getTextSize(msg,
                                 fontHeight,
                                 thickness,
                                 &baseline);

		if(thickness > 0){
			baseline += thickness;
		}
		
		// center the text
		Point textOrg((img.cols - textSize.width) / 2,
					  (img.rows + textSize.height) / 2);
			  
		// draw the box
		rectangle(img, textOrg + Point(0, baseline),
				  textOrg + Point(textSize.width, -textSize.height),
				  Scalar(0,255,0),1,8);

		// ... and the baseline first
		line(img, textOrg + Point(0, thickness),
			 textOrg + Point(textSize.width, thickness),
			 Scalar(0, 0, 255),1,8);
        
    
		ft2->putText(img, msg, textOrg, fontHeight,
					 Scalar::all(255), thickness, linestyle, true );
		
		ft2->putText(img, msg2, Point(100, 100), fontHeight,
					 Scalar(255, 0, 0), thickness, linestyle, true );

					 
		imwrite(argv[2], img);
		ft2.release();
		
    } catch (Exception e) {
        cout << e.what() << endl;
    }
	
    return 0;
}
