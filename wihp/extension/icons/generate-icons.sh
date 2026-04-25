#!/bin/bash
# Generate PNG icons from SVG
# Requires: inkscape or imagemagick

# Using Inkscape
if command -v inkscape &> /dev/null; then
    inkscape -w 16 -h 16 icon.svg -o icon16.png
    inkscape -w 48 -h 48 icon.svg -o icon48.png
    inkscape -w 128 -h 128 icon.svg -o icon128.png
    echo "Icons generated with Inkscape"
    exit 0
fi

# Using ImageMagick
if command -v convert &> /dev/null; then
    convert -background none -resize 16x16 icon.svg icon16.png
    convert -background none -resize 48x48 icon.svg icon48.png
    convert -background none -resize 128x128 icon.svg icon128.png
    echo "Icons generated with ImageMagick"
    exit 0
fi

# Using rsvg-convert
if command -v rsvg-convert &> /dev/null; then
    rsvg-convert -w 16 -h 16 icon.svg > icon16.png
    rsvg-convert -w 48 -h 48 icon.svg > icon48.png
    rsvg-convert -w 128 -h 128 icon.svg > icon128.png
    echo "Icons generated with rsvg-convert"
    exit 0
fi

echo "Please install inkscape, imagemagick, or librsvg to generate icons"
echo "Or manually convert icon.svg to icon16.png, icon48.png, icon128.png"
