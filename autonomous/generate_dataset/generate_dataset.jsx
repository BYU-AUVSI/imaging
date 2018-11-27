// In order to see this in File > Scripts menu, place in Photoshop's Presets/Scripts folder

function setBackgroundColor(hexCodeString)
{
    var currALayer = baseDoc.activeLayer; // Get current active layer
    // set your color as background color
    backgroundColor.rgb.hexValue = hexCodeString;
    baseDoc.activeLayer = baseDoc.layers[baseDoc.layers.length - 1] // Make last layer active (background)
    var aLayer = baseDoc.activeLayer; // Get active layer
    if (aLayer.isBackgroundLayer)
    {
        baseDoc.selection.fill(backgroundColor, ColorBlendMode.NORMAL, 100, false); // Fill background
    }
    baseDoc.activeLayer = currALayer; // Set the current active layer back to what is was
}

function doesLayerExist(name)
{
    var layers = baseDoc.layers;
    for (var i = 0; i < layers.length; i++)
    {
        if (layers[i].name === name) return true;
    }
    return false;
}

function hexToRgb(hex)
{
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}

// theArray: array of x, y points of the shape
// hexCodeString: string of the hex code resembling the color of the shape
function drawShape(shapeArray, hexCodeString)
{
    var doc = app.activeDocument;

    var lineArray = [];

    var layerName = "Color Fill 1"; // Name of the shape when it is filled
    if (doesLayerExist(layerName)) // If the old shape exists, remove it
    {
        doc.layers.getByName(layerName).remove();
    }

    for (var i = 0; i < shapeArray.length; i++)
    {

        lineArray[i] = new PathPointInfo();

        lineArray[i].kind = PointKind.CORNERPOINT; // Can be CORNERPOINT or SMOOTHPOINT

        lineArray[i].anchor = [Number(shapeArray[i][0]), Number(shapeArray[i][1])];

        $.writeln(shapeArray[i].join("___") + ("\n"))

        lineArray[i].leftDirection = lineArray[i].anchor;

        lineArray[i].rightDirection = lineArray[i].anchor;

    }


    var lineSubPathArray = new SubPathInfo();

    lineSubPathArray.closed = true;

    lineSubPathArray.operation = ShapeOperation.SHAPEADD;

    lineSubPathArray.entireSubPath = lineArray;

    var myPathItem = doc.pathItems.add("myPath", [lineSubPathArray]);

    var desc88 = new ActionDescriptor();

    var ref60 = new ActionReference();

    ref60.putClass(stringIDToTypeID("contentLayer"));

    desc88.putReference(charIDToTypeID("null"), ref60);

    var desc89 = new ActionDescriptor();

    var desc90 = new ActionDescriptor();

    var desc91 = new ActionDescriptor();

    desc91.putDouble(charIDToTypeID("Rd  "), hexToRgb(hexCodeString).r); // R

    desc91.putDouble(charIDToTypeID("Grn "), hexToRgb(hexCodeString).g); // G

    desc91.putDouble(charIDToTypeID("Bl  "), hexToRgb(hexCodeString).b); // B


    var id481 = charIDToTypeID("RGBC");

    desc90.putObject(charIDToTypeID("Clr "), id481, desc91);

    desc89.putObject(charIDToTypeID("Type"), stringIDToTypeID("solidColorLayer"), desc90);

    desc88.putObject(charIDToTypeID("Usng"), stringIDToTypeID("contentLayer"), desc89);

    executeAction(charIDToTypeID("Mk  "), desc88, DialogModes.NO);


    myPathItem.remove();
}

function rotateShape(cx, cy, shapeArray, angle)
{
    var radians = (Math.PI / 180) * angle;
    var cos = Math.cos(radians);
    var sin = Math.sin(radians);
    var rotatedShape = [];
    for (var i = 0; i < shapeArray.length; i++)
    {
        var x = shapeArray[i][0];
        var y = shapeArray[i][1];
        var nx = (cos * (x - cx)) + (sin * (y - cy)) + cx;
        var ny = (cos * (y - cy)) - (sin * (x - cx)) + cy;
        rotatedShape.push([nx, ny]);
    }

    return rotatedShape;
}

// x: x coordinate of the center of the triangle
// y: y coordinate of the center of the triangle
// b: base of the triangle
// h: height of the triangle
function getTriangle(x, y, b, h)
{
    var b2 = b * 0.5;
    var h2 = h * 0.5;
    return [[x, y - h2], [x + b2, y + h2], [x - b2, y + h2]];
}

// x: x coordinate of the center of the square
// y: y coordinate of the center of the square
// w: width of the square
function getSquare(x, y, w)
{
    var w2 = w * 0.5;
    return [[x - w2, y - w2], [x + w2, y - w2], [x + w2, y + w2], [x - w2, y + w2]];
}

// x: x coordinate of the center of the rectangle
// y: y coordinate of the center of the rectangle
// w: width of the rectangle
// h: height of the rectangle
function getRectangle(x, y, w, h)
{
    var w2 = w * 0.5;
    var h2 = h * 0.5;
    return [[x - w2, y - h2], [x + w2, y - h2], [x + w2, y + h2], [x - w2, y + h2]];
}

// x: x coordinate of the center of the trapezoid
// y: y coordinate of the center of the trapezoid
// w1: width of the top of the trapezoid
// w2: width of the bottom of the trapezoid
// h: height of the trapezoid
function getTrapezoid(x, y, w1, w2, h)
{
    var w12 = w1 * 0.5;
    var w22 = w2 * 0.5;
    var h2 = h * 0.5;
    return [[x - w12, y - h2], [x + w12, y - h2], [x + w22, y + h2], [x - w22, y + h2]];
}

// x: x coordinate of the center of the pentagon
// y: y coordinate of the center of the pentagon
// w: width of the pentagon
function getPentagon(x, y, w)
{
    var h = w * 0.95;
    var w2 = w * 0.5;
    var h2 = h * 0.5;
    var s = w / 1.618; // Length of side of pentagon
    var s2 = s * 0.5;
    var y_corner = Math.sqrt(Math.pow(s, 2) - Math.pow(h2, 2)) + (h2 - w2)
    return [[x, y - h2], [x + w2, y_corner], [x + s2, y + h2], [x - s2, y + h2], [x - w2, y_corner]];
}

// x: x coordinate of the center of the hexagon
// y: y coordinate of the center of the hexagon
// w: width of the hexagon
function getHexagon(x, y, w)
{
    var w2 = w * 0.5; // length of the side of a hexagon
    var w4 = w * 0.25;
    var h = 2 * w4 * Math.sqrt(3); // 30, 60, 90 triangle rule
    var h2 = h * 0.5;
    return [[x + w4, y - h2], [x + w2, y], [x + w4, y + h2], [x - w4, y + h2], [x - w2, y], [x - w4, y - h2]];
}

// x: x coordinate of center of polygon
// y: y coordinate of center of polygon
// numSides: number of sides of polygon
// r: radius of polygon
// theta: degrees by which the polygon is rotated
// if theta is 0, the first point is directly right of the center
function getPolygon(x, y, numSides, r, theta)
{
    var a = [];
    theta = theta * Math.PI / 180;
    for (var i = 0; i < numSides; i++)
    {
        var x_coor = r * Math.cos(2 * Math.PI * i / numSides - theta) + x;
        var y_coor = r * Math.sin(2 * Math.PI * i / numSides - theta) + y;
        a.push([x_coor, y_coor]);
    }
    return a;
}

function getCross(x, y, w)
{
    var h = w;
    var w2 = w * 0.5;
    var w4 = w * 0.125;
    var h2 = h * 0.5;
    var h4 = h * 0.125;
    return [
        [x - w4, y - h2], [x + w4, y - h2], [x + w4, y - h4], [x + w2, y - h4],
        [x + w2, y + h4], [x + w4, y + h4], [x + w4, y + h2], [x - w4, y + h2],
        [x - w4, y + h4], [x - w2, y + h4], [x - w2, y - h4], [x - w4, y - h4]
    ];
}

// x: x coordinate of the center of the star
// y: y coordinate of the center of the star
// w: width of the star
function getStar(x, y, w)
{
    var h = w;
    var w2 = w * 0.5;
    var h2 = h * 0.5;
    return [[(x), (y - h2)], [(x + 0.235 * w2), (y - 0.235 * h2)], [(x + w2), (y - 0.235 * h2)], [(x + 0.38 * w2), (y + 0.235 * h2)],
        [(x + 0.62 * w2), (y + h2)], [(x), (y + 0.52 * h2)], [(x - 0.6167 * w2), (y + h2)], [(x - 0.38 * w2), (y + 0.235 * h2)],
        [(x - w2), (y - 0.235 * h2)], [(x - 0.235 * w2), (y - 0.235 * h2)]];
}

function getQuarterCircle(x, y, r)
{
    var a = [];
    var r2 = (4 * r) / (3 * Math.PI); // Center of quarter circle
    a.push([x - r2, y - r2]); // Corner
    var numSides = 5 * DIM_; // How defined the curve is
    for (var i = 0; i < numSides; i++)
    {
        var x_coor = r * Math.cos(Math.PI / 2.0 * i / numSides) + x - r2;
        var y_coor = r * Math.sin(Math.PI / 2.0 * i / numSides) + y - r2;
        a.push([x_coor, y_coor]);
    }
    return a;
}

function getSemiCircle(x, y, r)
{
    var a = [];
    var r2 = (4 * r) / (3 * Math.PI); // Center of semi circle
    var numSides = 10 * DIM_; // How defined the curve is
    for (var i = 0; i < numSides; i++)
    {
        var x_coor = r * Math.cos(Math.PI * i / numSides) + x;
        var y_coor = r * Math.sin(Math.PI * i / numSides) + y - r2;
        a.push([x_coor, y_coor]);
    }
    return a;
}

const DIM_ = 100; // dimension of sides for the square image
const X_CENTER_ = DIM_ * 0.5;
const Y_CENTER_ = DIM_ * 0.5;
const ROTATE_STEP_ = 45; //degrees to rotate the letter for each image
const BLUR_LEVELS_ = 3; // how many blurs to apply 
const BLUR_STEP_ = 2; // amount of blur to apply at each level
//const BASE_SAVE_DIR_ = '~/Desktop/Generated/'; //base directory to save into
const BASE_SAVE_DIR_ = 'C:\\Users\\Brandon\\Documents\\imaging\\autonomous\\generate_dataset\\pics\\' // base directory to save into

var isTesting = true;

var alphabet = [];
if (isTesting)
{
    alphabet = "A".split(""); // test size alphabet
}
else
{
    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".split("");
}
var dimHalf = DIM_ / 2;

// set script units to pixels:
preferences.rulerUnits = Units.PIXELS;
var baseDoc = app.documents.add(DIM_, DIM_); // create new document


var whiteArrayVerbose = ["white",
    "ffffff", "f7f7f7", "f0f0f0", "eaeaea", "d9d9d9"]; // 5 shades

var whiteArray = ["white", "ffffff", "eaeaea"];

var blackArrayVerbose = ["black",
    "000000", "050505", "0a0a0a", "101010", "151515", "1a1a1a", "202020", "252525", "2a2a2a"]; // 9 shades

var blackArray = ["black", "000000", "151515"];

var grayArrayVerbose = ["gray",
    "505050", "555555", "5a5a5a", "606060", "656565", "6a6a6a", "707070", "757575", "7a7a7a",
    "808080", "858585", "8a8a8a", "909090", "959595", "9a9a9a", "a0a0a0", "a5a5a5"]; // 17 shades

var grayArray = ["gray", "606060", "909090"];

var redArrayVerbose = ["red",
    "ff0000", "ff2a00", "ff0033", "ff3333", "ff0066", "ff3366", "ff6666", "ff0099", "ff3399", "ff6699", "ff9999", "ffcccc",
    "cc0000", "cc1000", "cc0033", "cc3333", "cc0066", "cc3366",
    "990000", "990033", "993333",
    "660000"]; // 22 shades

var redArray = ["red", "ff0000", "cc3333"];

var blueArrayVerbose = ["blue",
    "0000ff", "3333ff", "0033ff", "3300ff", "6666ff", "0066ff", "0080ff", "0099ff", "3399ff", "6699ff",
    "00ccff", "33ccff", "66b2ff", "66ccff",
    "0000cc", "3300cc", "0033cc", "3333cc", "0066cc", "3366cc", "0099cc", "3399cc", "6699cc",
    "000099", "330099", "003399", "333399", "006699", "336699", "009999", "339999",
    "000066", "003366"]; // 33 shades

var blueArray = ["blue", "0000ff", "3333cc"];

var greenArrayVerbose = ["green",
    "00ff00", "33ff00", "00ff33", "33ff33", "66ff00", "00ff66", "66ff66", "00ff99", "99ff00", "99ff99", "00ffcc",
    "00cc00", "33cc00", "00cc33", "33cc33", "66cc00", "00cc66", "66cc66", "00cc99", "99ff00",
    "009900", "339900", "009933", "339933", "669900", "009966", "669966",
    "006600", "336600", "006633", "336633"]; // 31 shades

var greenArray = ["green", "00ff00", "33cc33"];

var yellowArrayVerbose = ["yellow",
    "ffff00", "ffff33", "ffff66", "ffff99", "ffffcc",
    "ffcc00", "ffcc33", "ffcc66",
    "cccc00", "cccc33", "cccc66", "cccc99",
    "cc9900", "cc9933",
    "999900", "999933",
    "666600", "666633"]; // 18 shades

var yellowArray = ["yellow", "ffff00", "cccc33"];

var purpleArrayVerbose = ["purple",
    "ff00ff", "ff33ff", "ff66ff", "ff99ff",
    "cc00ff", "cc33ff", "cc66ff", "cc99ff",
    "9900ff", "9933ff", "9966ff", "9999ff",
    "9900cc", "9933cc", "9966cc",
    "6600cc", "6633cc", "6666cc"]; // 18 shades

var purpleArray = ["purple", "cc00ff", "9933cc"];

var orangeArrayVerbose = ["orange",
    "ffbb00", "ffbb33", "ffbb66",
    "ff9900", "ff9933", "ff9966",
    "ff6600", "ff6633",
    "ff4500", "ff4533",
    "cc6633"]; // 10 shades

var orangeArray = ["orange", "ffa500", "ff6600"];

var brownArrayVerbose = ["brown",
    "8b4513", "a0522d", "cd853f", "d2691e",
    "996600",
    "cc6600"]; // 6 shades

var brownArray = ["brown", "8b4513", "a0522d"];

var colorArrays = [];
if (isTesting)
{
    colorArrays = [[whiteArray[0], whiteArray[1]], [blackArray[0], blackArray[1]]]; // test size color

}
else
{
    colorArrays = [whiteArray, blackArray, grayArray, redArray, blueArray,
        greenArray, yellowArray, purpleArray, orangeArray, brownArray];
}


var triangleB = DIM_ * 0.75;      // base of triangle
var triangleH = triangleB;      // height of triangle
var squareR = DIM_ * 0.45;    // radius of square
var pentagonR = DIM_ * 0.45;    // radius of pentagon
var hexagonR = DIM_ * 0.45;    // radius of hexagon
var heptagonR = DIM_ * 0.45;    // radius of heptagon
var octagonR = DIM_ * 0.45;    // radius of octagon
var crossW = DIM_ * 0.9;     // width of cross
var starW = DIM_ * 0.9;     // width of star
var quarterCircleR = DIM_ * 0.75; // radius of quarter circle
var semiCircleR = DIM_ * 0.45;    // radius of semi circle

var shapesArray = [];
if (isTesting)
{
    shapesArray = [["triangle-0", getTriangle(X_CENTER_, Y_CENTER_, triangleB, triangleH)]]; // Triangle pointing up
}
else
{
    shapesArray =
        [
            ["triangle-0", getTriangle(X_CENTER_, Y_CENTER_, triangleB, triangleH)], // Triangle pointing up
            // ["triangle-90", rotateShape(X_CENTER_, Y_CENTER_, getTriangle(X_CENTER_, Y_CENTER_, triangleB, triangleH), 90)],  // Triangle pointing left
            // ["triangle-180", rotateShape(X_CENTER_, Y_CENTER_, getTriangle(X_CENTER_, Y_CENTER_, triangleB, triangleH), 180)],// Triangle pointing down
            // ["triangle-270", rotateShape(X_CENTER_, Y_CENTER_, getTriangle(X_CENTER_, Y_CENTER_, triangleB, triangleH), 270)],// Triangle pointing right
            ["square", getPolygon(X_CENTER_, Y_CENTER_, 4, squareR, 45)],    // Flat side up and down
            ["square-45", getPolygon(X_CENTER_, Y_CENTER_, 4, squareR, 0)]   // Diamond shape
                ["rectangle-long", getRectangle(X_CENTER_, Y_CENTER_, DIM_ * 0.5, DIM_)],             // Long rectangle
            ["rectangle-wide", getRectangle(X_CENTER_, Y_CENTER_, DIM_, DIM_ * 0.5)],             // Wide rectangle
            ["trapezoid-up", getTrapezoid(X_CENTER_, Y_CENTER_, DIM_ * 0.5, DIM_, DIM_ * 0.5)],     // Short side up
            ["trapezoid-down", getTrapezoid(X_CENTER_, Y_CENTER_, DIM_, DIM_ * 0.5, DIM_ * 0.5)],   // Short side down
            ["pentagon-0", getPolygon(X_CENTER_, Y_CENTER_, 5, pentagonR, 90)],        // Pointing up
            // ["pentagon-90", getPolygon(X_CENTER_, Y_CENTER_, 5, pentagonR, 0)],        // Pointing right
            // ["pentagon-180", getPolygon(X_CENTER_, Y_CENTER_, 5, pentagonR, -90)],     // Pointing down
            // ["pentagon-270", getPolygon(X_CENTER_, Y_CENTER_, 5, pentagonR, 180)],     // Pointing left
            ["hexagon-0", getPolygon(X_CENTER_, Y_CENTER_, 6, hexagonR, 0)],    // Flat side up and down
            // ["hexagon-90", getPolygon(X_CENTER_, Y_CENTER_, 6, hexagonR, 90)],  // Flat side left and right
            // ["heptagon-0", getPolygon(X_CENTER_, Y_CENTER_, 7, heptagonR, 90)],        // Pointing up
            // ["heptagon-180", getPolygon(X_CENTER_, Y_CENTER_, 7, heptagonR, -90)],     // Pointing down
            ["octagon-0", getPolygon(X_CENTER_, Y_CENTER_, 8, octagonR, 22.5)],       // Flat side up and down
            // ["octagon-23", getPolygon(X_CENTER_, Y_CENTER_, 8, octagonR, 0)],         // Flat side left and right
            ["cross-0", getCross(X_CENTER_, Y_CENTER_, crossW)],            // Regular cross
            ["cross-45", rotateShape(X_CENTER_, Y_CENTER_, getCross(X_CENTER_, Y_CENTER_, crossW), 45)],    // Cross rotated like an X
            ["star-0", getStar(X_CENTER_, Y_CENTER_, starW)],                                               // Star pointing up
            // ["star-90", rotateShape(X_CENTER_, Y_CENTER_, getStar(X_CENTER_, Y_CENTER_, starW), -90)],     // Star pointing right
            // ["star-180", rotateShape(X_CENTER_, Y_CENTER_, getStar(X_CENTER_, Y_CENTER_, starW), 180)],     // Star pointing down
            // ["star-270", rotateShape(X_CENTER_, Y_CENTER_, getStar(X_CENTER_, Y_CENTER_, starW), 90)],     // Star pointing left
            ["quarterCircle-0", getQuarterCircle(X_CENTER_, Y_CENTER_, quarterCircleR)],
            // ["quarterCircle-90",rotateShape(X_CENTER_, Y_CENTER_, getQuarterCircle(X_CENTER_, Y_CENTER_, quarterCircleR), 90)],
            // ["quarterCircle-180",rotateShape(X_CENTER_, Y_CENTER_, getQuarterCircle(X_CENTER_, Y_CENTER_, quarterCircleR), 180)],
            // ["quarterCircle-270",rotateShape(X_CENTER_, Y_CENTER_, getQuarterCircle(X_CENTER_, Y_CENTER_, quarterCircleR), 270)],
            ["semiCircle-0", getSemiCircle(X_CENTER_, Y_CENTER_, semiCircleR)],
            // ["semiCircle-90",rotateShape(X_CENTER_, Y_CENTER_, getSemiCircle(X_CENTER_, Y_CENTER_, semiCircleR), 90)],
            // ["semiCircle-180",rotateShape(X_CENTER_, Y_CENTER_, getSemiCircle(X_CENTER_, Y_CENTER_, semiCircleR), 180)],
            // ["semiCircle-270",rotateShape(X_CENTER_, Y_CENTER_, getSemiCircle(X_CENTER_, Y_CENTER_, semiCircleR), 270)],
        ];
}

var maxAngle;
if (isTesting)
{
    maxAngle = 90;
}
else
{
    maxAngle = 360;
}

var jpegOpts = new JPEGSaveOptions;
jpegOpts.FormatOptions = FormatOptions.STANDARDBASELINE;
jpegOpts.quality = 7;

// for (var j = 0; j < BLUR_LEVELS_; j++) {

// textLayer.applyMotionBlur(45, BLUR_STEP_);

for (var i = 0; i < alphabet.length; i++)
{
    for (var shapeI = 0; shapeI < colorArrays.length; shapeI++)
    {
        var shapeColorName = colorArrays[shapeI][0]; // name of color for naming the file
        if (shapeColorName === "white") // If the shape color is white change the background color to black
        {
            setBackgroundColor("000000")
        }
        else // Else white background
        {
            setBackgroundColor("ffffff")
        }
        for (var shapeJ = 0; shapeJ < colorArrays[shapeI].length; shapeJ++)
        {
            if (shapeJ === 0)
            {
                shapeJ++; // shapeJ[0] is the name of the color, not a hex code
            }
            for (var shapeK = 0; shapeK < shapesArray.length; shapeK++)
            {
                var shapeName = shapesArray[shapeK][0]; // Name of shape for saving file
                drawShape(shapesArray[shapeK][1], colorArrays[shapeI][shapeJ]);
                for (var textI = 0; textI < colorArrays.length; textI++)
                {
                    var textColorName = colorArrays[textI][0]; // name of color for naming the file
                    for (var textJ = 0; textJ < colorArrays[textI].length; textJ++)
                    {
                        if (textJ === 0)
                        {
                            textJ++; // textJ[0] is the name of the color, not a hex code
                        }

                        if (textI === shapeI) // If shape color and text color are the same, skip to the next color
                        {
                            break;
                        }

                        // setup the text layer
                        var textLayer = baseDoc.artLayers.add();
                        textLayer.kind = LayerKind.TEXT;
                        var text = textLayer.textItem;
                        text.size = 28;
                        var textColor = new SolidColor(); // create text color

                        text.contents = alphabet[i]; // write letter
                        textLayer.move(baseDoc, ElementPlacement.PLACEATBEGINNING); // Place text in the front

                        // center text
                        var bounds = textLayer.bounds; //If the font isn't monospaced, our bounds will change with each letter
                        // bound[0], bound[1] is the top left corner
                        // -bounds resets image to top left corner
                        // -bounds[0] + DIM_/2 places left edge in center
                        // (bounds[2] - bounds[0]) / 2 figures out the center of the content in the layer (ie: the letter)
                        //      and subtracts that so the center of the layer content is in the center of the document
                        var xTranslate = (-bounds[0] + dimHalf) - ((bounds[2] - bounds[0]) / 2);
                        var yTranslate = (-bounds[1] + dimHalf) - ((bounds[3] - bounds[1]) / 2);
                        textLayer.translate(xTranslate, yTranslate);

                        // Set text color
                        textColor.rgb.hexValue = colorArrays[textI][textJ];
                        textLayer.textItem.color = textColor;

                        textLayer.rasterize(RasterizeType.TEXTCONTENTS);

                        for (var j = 0; j < BLUR_LEVELS_; j++)
                        {
                            // rotate around
                            var angle = 0; //keeps track of how far we're rotated
                            do
                            {
                                // true saves the jpeg as a copy
                                baseDoc.saveAs(new File(BASE_SAVE_DIR_ + shapeColorName + shapeJ + "-" + shapeName + "-" + textColorName + textJ + "-" + alphabet[i] + "-blur" + j + "-" + angle + ".jpeg"), jpegOpts, true);
                                angle += ROTATE_STEP_;
                                textLayer.rotate(ROTATE_STEP_, AnchorPosition.MIDDLECENTER);
                            } while (angle < maxAngle);

                            textLayer.applyMotionBlur(45, BLUR_STEP_);
                        } // blur levels
                        textLayer.remove();
                    } // textJ
                }// textI
            }// shapeK
        }// shapeJ
    } // shapeI
} // alphabet
// }
