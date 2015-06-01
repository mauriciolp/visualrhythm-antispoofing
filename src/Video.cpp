/*------------------------------------------------------------------------------------------------*\
    This file contains material supporting chapter 10 of the cookbook:
    Computer Vision Programming using the OpenCV Library.
    by Robert Laganiere, Packt Publishing, 2011.

    This program is free software; permission is hereby granted to use, copy, modify,
    and distribute this source code, or portions thereof, for any purpose, without fee,
    subject to the restriction that the copyright notice may not be removed
    or altered from any source or altered source distribution.
    The software is released on an as-is basis and without any warranties of any kind.
    In particular, the software is not guaranteed to be fault-tolerant or free from failure.
    The author disclaims all warranties with regard to this software, any use,
    and any consequent failure, is purely the responsibility of the user.

    Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------------*/

#include "Video.h"
#include "VisualRhythm.h"
using namespace std;
using namespace cv;

Video::Video() {
    this->stop = false;
    this->callIt = false;
    this->delay = 0;
    this->fnumber = 0;
    this->frameToStop = -1;
    this->frameProcessor = 0;
    this->processo = 0;
}

Video::~Video() {
}

bool Video::setInput(string filename) {
    fnumber = 0;
    capture.release();

    // CUSTOM
    input_filename = filename;
    return capture.open(filename);
}

bool Video::setOutput(const std::string &filename, int codec, double framerate,
        cv::Size size, bool isColor) {

    outputFile = filename;

    char c[4];

    // use same codec as input
    if (codec == -1) {
        codec = getCodec(c);
    }

    // Abre o vídeo de saída
    return writer.open(outputFile, codec, framerate, size, isColor);
}

void Video::setFrameProcessor(
        void (*frameProcessingCallback)(cv::Mat&, cv::Mat&)) {

    frameProcessor = 0;

    processo = frameProcessingCallback;
    callProcess();
}

void Video::setFrameProcessor(FrameProcessor* frameProcessorPtr) {

    processo = 0;

    frameProcessor = frameProcessorPtr;
    callProcess();
}

void Video::stopAtFrameNo(long frame) {
    this->frameToStop = frame;
}

void Video::callProcess() {
    this->callIt = true;
}

void Video::dontCallProcess() {
    this->callIt = false;
}

void Video::displayInput(string wn) {
    windowNameInput = wn;
    namedWindow(windowNameInput, CV_WINDOW_AUTOSIZE);
}

void Video::displayOutput(string wn) {
    windowNameOutput = wn;
    namedWindow(windowNameOutput, CV_WINDOW_AUTOSIZE);
}

void Video::dontDisplay() {
    destroyWindow(windowNameInput);
    destroyWindow(windowNameOutput);
    windowNameInput.clear();
    windowNameOutput.clear();
}

void Video::setDelay(int delay) {
    this->delay = delay;
}

long Video::getNumberOfProcessedFrames() {
    return fnumber;
}

Size Video::getFrameSize() {

    return cv::Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH),
            (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));
}

long Video::getFrameNumber() {
    long f = capture.get(CV_CAP_PROP_POS_FRAMES);
    return f;
}

double Video::getFrameRate() {
    double r = capture.get(CV_CAP_PROP_FPS);
    return r;
}

long Video::getTotalFrameCount() {
    long t = capture.get(CV_CAP_PROP_FRAME_COUNT);
    return t;
}

int Video::getCodec(char codec[4]) {

    union {
        int value;
        char code[4];
    } returned;

    returned.value = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));

    codec[0] = returned.code[0];
    codec[1] = returned.code[1];
    codec[2] = returned.code[2];
    codec[3] = returned.code[3];

    return returned.value;
}

void Video::stopIt() {
    this->stop = true;
}

bool Video::isStopped() const {
    return stop;
}

bool Video::isOpened() {
    return capture.isOpened();
}

long Video::getFrameToStop() const {
    return frameToStop;
}

void Video::setFrameToStop(long frameToStop) {
    this->frameToStop = frameToStop;
}

void Video::saveFirstFrame(std::string fileImage) {
    Mat frame;
    if (!isOpened()) {
        cout << "Não foi possível abrir o arquivo de vídeo" << endl;
        return;
    }
    if (!readNextFrame(frame)) {
        cout << "Erro: readNextFrame" << endl;
        return;
    }

    imwrite(fileImage, frame);

    capture.release();
    frame.release();
}

bool Video::readNextFrame(cv::Mat& frame) {
    return capture.read(frame);
}

void Video::writeNextFrame(cv::Mat& frame) {
    writer.write(frame);
}

void Video::setOutputFilePath(std::string image_fpath) {
	this->image_fpath = image_fpath;
}

void Video::run() {
    cv::Mat frame;
    cv::Mat output;

    // CUSTOM {
    int total_vr, curr_vr=1, stride=50, window_size=frameToStop, window_begin;
    ostringstream convert;   // stream used for the conversion of int to str
    std::string output_file;

    float aspect_ratio;
    Size dsize;
    VisualRhythm *vr_temp;
    // }

    if (!isOpened()){
		cout << "*Unable to open video file* not ";
        return;
	}

    stop = false;

    // do a while already dividing getTotalFrameCount/50 = total_VR?
    // decision if should stop before last window is getTotalFrameCount%50 > 10 (window size)
    // curr_VR = 1

    // *Create a flag for this new type of proccess, and mantain funcionalities*
    //~ cout << "getTotalFrameCount(): " << getTotalFrameCount() << endl;

    total_vr = getTotalFrameCount()/stride;
    window_begin = (curr_vr-1)*stride + 1;

    while (curr_vr <= total_vr) {

        if (!readNextFrame(frame))
            break;

        vr_temp = (VisualRhythm*) frameProcessor;

        aspect_ratio = 1.*frame.size().height / frame.size().width;
		//~ For Vertical only
		dsize = Size(int(vr_temp->getWidth() * window_size / aspect_ratio), vr_temp->getWidth()*window_size);

        cv::resize(frame,frame,dsize);

        if (windowNameInput.length() != 0)
            cv::imshow(windowNameInput, frame);

		// window_end = window_begin + window_size

		if (window_begin <= getFrameNumber() and getFrameNumber()
			< window_begin + window_size){
			//~ cout << "getFrameNumber(): " << getFrameNumber() << endl;

			if (callIt) {
				if (processo)
					processo(frame, output);
				else if (frameProcessor)
					frameProcessor->process(frame, output);
			} else {
				output = frame;
			}

			if (outputFile.length() != 0)
				writeNextFrame(output);

			if (windowNameOutput.length() != 0)
				cv::imshow(windowNameOutput, output);

			// *IMPORTANT* Line commented because some videos would freeze here
			if (delay > 0 && cv::waitKey(delay) >= 0)
			//~ if (delay >= 0 && cv::waitKey(delay) >= 0)
				stopIt();
		}

		if (getFrameNumber() >= window_begin + window_size){

			convert.str("");
			convert.clear();
			convert << curr_vr;      // insert the textual representation of 'Number' in the characters in the stream

			// TODO format input_filename to get only the file.
			// For evaluation only, the parameter input_filename must be the name exactly of the file
			// a gambs would be replace the '/' by '_', so it could be created the file

			// Temporary workaround for replacing '/' for '_' in input file name
			std::size_t found = input_filename.rfind('/');
			while (found!=std::string::npos){
				input_filename.replace (found,1,"_");
				found = input_filename.rfind('/');
			}

			// TODO insert curr_vr before extension of output OR static png

			output_file = image_fpath + "/" + input_filename + "_" + convert.str() + ".png";

			//~ cout << '\n' << output_file;

			VisualRhythm *vr;

			vr = (VisualRhythm*) frameProcessor;

			vr->setOutputFileName(output_file.c_str() );
			vr->saveVisualRhythm();

			//~ cout << "\n vr->getHeight(): " << vr->getHeight() << '\n';
			//~ cout << "\n vr->getWidth(): " << vr->getWidth() << '\n';
			//~ cout << "\n frameToStop: " << frameToStop << '\n';

			//~ vr->createVisualRhythm(Mat(vr->getHeight(), vr->getWidth() * frameToStop, CV_8U));
			vr->createVisualRhythm(Mat(vr->getHeight(), vr->getWidth() * frameToStop, CV_8UC3));

			curr_vr++;
			window_begin = (curr_vr-1)*stride + 1;
		}
    }

    //~ while (!isStopped()) {
//~
        //~ if (!readNextFrame(frame))
            //~ break;
//~
        //~ if (windowNameInput.length() != 0)
            //~ cv::imshow(windowNameInput, frame);
//~
        //~ if (callIt) {
            //~ if (processo)
                //~ processo(frame, output);
            //~ else if (frameProcessor)
                //~ frameProcessor->process(frame, output);
//~
        //~ } else {
            //~ output = frame;
        //~ }
//~
        //~ if (outputFile.length() != 0)
            //~ writeNextFrame(output);
//~
        //~ if (windowNameOutput.length() != 0)
            //~ cv::imshow(windowNameOutput, output);
//~
        //~ if (delay >= 0 && cv::waitKey(delay) >= 0)
            //~ stopIt();
//~
        //~ if (frameToStop >= 0 && getFrameNumber() == frameToStop)
            //~ stopIt();
    //~ }
}
