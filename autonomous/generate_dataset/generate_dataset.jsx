// In order to see this in File > Scripts menu, place in Photoshop's Presets/Scripts folder

const DIM_ = 40; // dimension of sides for the square image
const ROTATE_STEP_ = 45; //degrees to rotate the letter for each image
const BLUR_LEVELS_ = 3; // how many blurs to apply 
const BLUR_STEP_ = 2; // amount of blur to apply at each level
//const BASE_SAVE_DIR_ = '~/Desktop/Generated/'; //base directory to save into
const BASE_SAVE_DIR_ = 'C:\\Users\\Brandon\\Documents\\Photoshop Script\\Pics\\' // base directory to save into

// var alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".split("");
var alphabet = "AB".split(""); // test size alphabet
var dimHalf = DIM_ / 2;

// set script units to pixels:
preferences.rulerUnits = Units.PIXELS;
var baseDoc = app.documents.add(DIM_, DIM_); // create new document

// setup the text layer
var textLayer = baseDoc.artLayers.add();
textLayer.kind = LayerKind.TEXT;
baseDoc.layers[1];
var text = textLayer.textItem;
text.size = 36;

// create a color of your choice
var BGcolor = new SolidColor();
BGcolor.rgb.red = 0;
BGcolor.rgb.green = 0;
BGcolor.rgb.blue = 255;

// set your color as background color
backgroundColor.rgb.hexValue = BGcolor.rgb.hexValue;
var bgLayer = baseDoc.layers[baseDoc.layers.length - 1]
if (bgLayer.isBackgroundLayer)
{
    baseDoc.selection.fill(backgroundColor, ColorBlendMode.NORMAL, 100, false);
}


// for (var j = 0; j < BLUR_LEVELS_; j++) {

// textLayer.applyMotionBlur(45, BLUR_STEP_);

for (var i = 0; i < alphabet.length; i++)
{
    text.contents = alphabet[i];

    // center::
    var bounds = textLayer.bounds; //If the font isn't monospaced, our bounds will change with each letter
    // bound[0], bound[1] is the top left corner
    // -bounds resets image to top left corner
    // -bounds[0] + DIM_/2 places left edge in center
    // (bounds[2] - bounds[0]) / 2 figures out the center of the content in the layer (ie: the letter)
    //      and subtracts that so the center of the layer content is in the center of the document
    var xTranslate = (-bounds[0] + dimHalf) - ((bounds[2] - bounds[0]) / 2);
    var yTranslate = (-bounds[1] + dimHalf) - ((bounds[3] - bounds[1]) / 2);
    textLayer.translate(xTranslate, yTranslate);
    
    var jpegOpts = new JPEGSaveOptions;
    jpegOpts.FormatOptions = FormatOptions.STANDARDBASELINE;
    jpegOpts.quality = 7;
    
    // rotate around
    var angle = 0; //keeps track of how far we're rotated
    do
    {
        // true saves the jpeg as a copy
        baseDoc.saveAs(new File(BASE_SAVE_DIR_ + alphabet[i] + "-" + angle + ".jpeg"), jpegOpts, true);
        angle += ROTATE_STEP_;
        textLayer.rotate(ROTATE_STEP_, AnchorPosition.MIDDLECENTER);
    } while (angle < 360);

}
// }
