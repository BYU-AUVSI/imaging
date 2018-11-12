// In order to see this in File > Scripts menu, place in Photoshop's Presets/Scripts folder

const DIM_ = 40; // dimension of sides for the square image
const ROTATE_STEP_ = 45; //degrees to rotate the letter for each image
const BLUR_LEVELS_ = 3; // how many blurs to apply 
const BLUR_STEP_ = 2; // amount of blur to apply at each level
//const BASE_SAVE_DIR_ = '~/Desktop/Generated/'; //base directory to save into
const BASE_SAVE_DIR_ = 'C:\\Users\\Brandon\\Documents\\imaging\\autonomous\\generate_dataset\\pics\\' // base directory to save into

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

// set your color as background color
backgroundColor.rgb.hexValue = "FF0000";
baseDoc.activeLayer = baseDoc.layers[baseDoc.layers.length - 1]
var aLayer = baseDoc.activeLayer;
if (aLayer.isBackgroundLayer)
{
    baseDoc.selection.fill(backgroundColor, ColorBlendMode.NORMAL, 100, false);
}


var whiteArray = ["white", "ffffff", "f7f7f7", "f0f0f0", "eaeaea", "d9d9d9"];

var blackArray = ["black", "000000", "050505", "0a0a0a", "101010", "151515", "1a1a1a", "202020", "252525", "2a2a2a"];
var grayArray = ["gray", "505050", "555555", "5a5a5a", "606060", "656565", "6a6a6a", "707070", "757575", "7a7a7a",
                 "808080", "858585", "8a8a8a", "909090", "959595", "9a9a9a", "a0a0a0", "a5a5a5"];

var redArray = ["red", "ff0000", "ff2a00", "ff0033", "ff3333", "ff0066", "ff3366", "ff6666", "ff0099", "ff3399", "ff6699", "ff9999", "ffcccc",
                "cc0000", "cc1000", "cc0033", "cc3333", "cc0066", "cc3366",
                "990000", "990033", "993333",
                "660000"];

var blueArray = ["blue", "0000ff", "3333ff", "0033ff", "3300ff", "6666ff", "0066ff", "0080ff", "0099ff", "3399ff", "6699ff",
                 "00ccff", "33ccff", "66b2ff", "66ccff",
                 "0000cc", "3300cc", "0033cc", "3333cc", "0066cc", "3366cc", "0099cc", "3399cc", "6699cc",
                 "000099", "330099", "003399", "333399", "006699", "336699", "009999", "339999",
                 "000066", "003366"];

var greenArray = ["green", "00ff00", "33ff00", "00ff33", "33ff33", "66ff00", "00ff66", "66ff66", "00ff99", "99ff00", "99ff99", "00ffcc",
                  "00cc00", "33cc00", "00cc33", "33cc33", "66cc00", "00cc66", "66cc66", "00cc99", "99ff00",
                  "009900", "339900", "009933", "339933", "669900", "009966", "669966",
                  "006600", "336600", "006633", "336633"];

var yellowArray = ["yellow", "ffff00", "ffff33", "ffff66", "ffff99", "ffffcc",
                   "ffcc00", "ffcc33", "ffcc66",
                   "cccc00", "cccc33", "cccc66", "cccc99",
                   "cc9900", "cc9933",
                   "999900", "999933",
                   "666600", "666633"];

var purpleArray  = ["purple", "ff00ff", "ff33ff", "ff66ff", "ff99ff",
                    "cc00ff", "cc33ff", "cc66ff", "cc99ff",
                    "9900ff", "9933ff", "9966ff", "9999ff",
                    "9900cc", "9933cc", "9966cc",
                    "6600cc", "6633cc", "6666cc"];

var orangeArray = ["orange", "ffbb00", "ffbb33", "ffbb66",
                   "ff9900", "ff9933", "ff9966",
                   "ff6600", "ff6633",
                   "ff4000", "ff4033",
                   "cc6633"];

var brownArray = ["brown", "996600",
                  "cc6600"];

// var colorArrays = [whiteArray, blackArray, grayArray, redArray, blueArray,
//                    greenArray, yellowArray, purpleArray, orangeArray, brownArray];

var colorArrays = [[blueArray[0], blueArray[1]], [whiteArray[0], whiteArray[1]], [greenArray[0], greenArray[1]]]; // test size color


// for (var j = 0; j < BLUR_LEVELS_; j++) {

// textLayer.applyMotionBlur(45, BLUR_STEP_);

for (var i = 0; i < alphabet.length; i++)
{
    for (var textI = 0; textI < colorArrays.length; textI++)
    {
        var textColorName = colorArrays[textI][0];
        for (var textJ = 0; textJ < colorArrays[textI].length; textJ++)
        {
            for (var bgI = 0; bgI < colorArrays.length; bgI++)
            {
                var bgColorName = colorArrays[bgI][0];
                for (var bgJ = 0; bgJ < colorArrays[bgI].length; bgJ++)
                {
                    if (textJ === 0)
                    {
                        textJ++; // textJ[0] is the name of the color, not a hex code
                    }
                    if (bgJ === 0)
                    {
                        bgJ++; // bgJ[0] is the name of the color, not a hex code
                    }
                    if (textI === bgI) // If background color and text color are the same, skip to the next color
                    {
                        break;
                    }

                    // create a color of your choice
                    var textColor = new SolidColor();
                    textColor.rgb.hexValue = colorArrays[textI][textJ];
                    textLayer.textItem.color = textColor;

                    text.contents = alphabet[i];

                    // set your color as background color
                    backgroundColor.rgb.hexValue = colorArrays[bgI][bgJ];
                    baseDoc.activeLayer = baseDoc.layers[baseDoc.layers.length - 1]
                    var aLayer = baseDoc.activeLayer;
                    if (aLayer.isBackgroundLayer)
                    {
                        baseDoc.selection.fill(backgroundColor, ColorBlendMode.NORMAL, 100, false);
                    }

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
                        baseDoc.saveAs(new File(BASE_SAVE_DIR_ + bgColorName + bgJ + "-bg-" + textColorName + textJ + "-" + alphabet[i] + "-" + angle + ".jpeg"), jpegOpts, true);
                        angle += ROTATE_STEP_;
                        textLayer.rotate(ROTATE_STEP_, AnchorPosition.MIDDLECENTER);
                    } while (angle < 360);
                } // bgJ
            }// bgI
        }// textJ
    } // textI
} // alphabet
// }
