#include "run_camera_calibration.hpp"

#include "CLI/CLI.hpp"

int main(int ac, char **av) {
	std::string framesPath;
	std::string patternPath;
	std::string outputFilepath;
	int32_t width;
	int32_t height;

	// Use CLI11 library to handle argument parsing
	CLI::App app{"Runs camera calibration from a single recording's frame reconstruction given provided sensor and "
				 "pattern info."};
	app.add_option("-f,--framesPath", framesPath,
		   "Path to file containing frames with patterns to be detected for calibration.")
		->required();
	app.add_option("-o,--outputFile", outputFilepath, "Path to file where to save the calibration.")->required();
	app.add_option("-p,--pattern", patternPath,
		   "Path to json file containing calibration pattern information (name, shape, dimensions)")
		->required();
	app.add_option("--width", width, "Number of pixels along the width of sensor.")->default_val(640);
	app.add_option("--height", height, "Number of pixels along the height of sensor.")->default_val(480);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	runCameraCalibration(framesPath, outputFilepath, patternPath, width, height);

	return EXIT_SUCCESS;
}
