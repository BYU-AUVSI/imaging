# Dataset Generators

These scripts are used to generate datasets for the various CNN classifiers. The
main objective is to generate lots of real-world data, without requiring hours
of flight time and images. Generated data should also represent what comes out of
the autonomous detector.

As you add scripts, please take 2 seconds to give a 1-2 sentence description of them below

## Scripts

`create_validation_set.py` Utility script. Given a dataset folder (structured: 
dataset/train/className/imageX.jpg), take a random percentage of images from each
class and put them into the validation set (ie: dataset/val/className/imageX.jpg).

`synthetic_shapeLetter_generator.py` Fully synthetic dataset generator that produces
all combinations of letter+shape in various colors. Unused for training at this time,
but kept for historical value ;)

`synthetic_letter_generator.py` Generates completely synthetic letter images using
various fonts, font sizes, rotations and blurs.

`shape_video_data_generator.py` Generates shape images from videos of targets, see it
in action [here](https://youtu.be/fyoo3Zcpb-k). This is the currently used script
as it allows quick data generation of real-world shapes in various orientations and angles.

`letter_video_data_generator.py` Similar to the shape video data generator, but
customized for letters. This script is currently used to augment the dataset
provided by the synthetic_letter_generator.

## Structure

`assets` Holds the various fonts used by the dataset generators