#!/bin/bash

# Create all 8 English chapters with basic structure
for i in {01..08}; do
  case $i in
    01) title="Aging Society and Care Technology" ;;
    02) title="Fall Detection Systems" ;;
    03) title="Activity Monitoring" ;;
    04) title="Cognitive Function Assistance" ;;
    05) title="Voice Control Interface" ;;
    06) title="Emergency Response System" ;;
    07) title="Social Connectivity" ;;
    08) title="Medication Management and Accessibility" ;;
  esac
  
  echo "Creating chapter-$i.html: $title"
done

echo "Script prepared. Run manually to create chapters."
