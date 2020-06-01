# Definition for the Silicion Eye Event Sensor (SEES) polarity event message type: The index origin is in the upper left corner
uint16 x # x-coordinate of the event (column index)
uint16 y # y-coordinate of the event (row index)
time ts # timestamp
bool polarity # The polarity of the event: True corresponds to positive polarity (brighter), false to negatative polarity (darker)
