#ifndef AUTOCOMPLETEGRAPH_KEYPOINTPRIORDETECTOR_07042017
#define AUTOCOMPLETEGRAPH_KEYPOINTPRIORDETECTOR_07042017

#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#if CV_MAJOR_VERSION == 2
// Old OpenCV 2 code goes here.
#include "opencv2/nonfree/features2d.hpp"
#else
// New OpenCV 3 code goes here.
#include "opencv2/xfeatures2d.hpp"
#endif


namespace AASS{
	namespace acg{

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class KeypointPriorDetector{
			
		protected:
			std::vector<std::tuple<cv::KeyPoint, cv::Mat> > keypoints_;

#if CV_MAJOR_VERSION == 2
            cv::SiftDescriptorExtractor extractor_;
#else
            cv::Ptr<cv::Feature2D> extractor_ = cv::xfeatures2d::SURF::create();
//            cv::xfeatures2d::SiftDescriptorExtractor extractor_;
#endif

			cv::FlannBasedMatcher matcher_;
			
		public:
			
            KeypointPriorDetector(){}
			void descriptors();
			
			
		private:
			
			void priorToMat();
			
			/**
			 * @brief extract SIFT descriptors from the image
			 */
			void extractDescriptors(const cv::Mat& img){
				std::vector<cv::KeyPoint> keypoints_1;
				cv::Mat descriptors_1;
#if CV_MAJOR_VERSION == 2
                extractor_.compute( img, keypoints_1, descriptors_1 );
#else
                extractor_->compute( img, keypoints_1, descriptors_1 );
#endif
				for(int i = 0 ; i < keypoints_1.size() ; ++i){
				}
				
			}
			
		};
	}
}
#endif
