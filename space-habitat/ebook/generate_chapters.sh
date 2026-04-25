#!/bin/bash
# This script will be used to check if we need to create additional chapters
# For now, let's just list what we have
echo "Korean chapters:"
ls -lh ko/
echo ""
echo "English chapters:"
ls -lh en/ 2>/dev/null || echo "English directory not yet created"
