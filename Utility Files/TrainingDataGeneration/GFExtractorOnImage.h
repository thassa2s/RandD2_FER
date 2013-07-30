#pragma once

#include "cvGaborWiki/CvGaborWiki.cpp"
#include "general.h"

class GFExtractorOnImage
{
public:
	GFExtractorOnImage(void);
	~GFExtractorOnImage(void);

	void extractFeaturesOnImage(IplImage *image,
								vector1Df &gFV);

private:
	void fillGaborFeatureVector(IplImage *magresp,
								vector1Df &gFV);
};
