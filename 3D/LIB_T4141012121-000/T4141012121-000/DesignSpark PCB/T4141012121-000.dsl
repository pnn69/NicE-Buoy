SamacSys ECAD Model
2059684/1077111/2.50/12/2/Connector

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c130_h80"
		(holeDiam 0.8)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.300) (shapeHeight 1.300))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.300) (shapeHeight 1.300))
	)
	(padStyleDef "s130_h80"
		(holeDiam 0.8)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.300) (shapeHeight 1.300))
		(padShape (layerNumRef 16) (padShapeType Rect)  (shapeWidth 1.300) (shapeHeight 1.300))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "T4140012121000" (originalName "T4140012121000")
		(multiLayer
			(pad (padNum 1) (padStyleRef s130_h80) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c130_h80) (pt -2.856, 0.000) (rotation 90))
			(pad (padNum 3) (padStyleRef c130_h80) (pt -4.100, -1.397) (rotation 90))
			(pad (padNum 4) (padStyleRef c130_h80) (pt -4.232, -3.262) (rotation 90))
			(pad (padNum 5) (padStyleRef c130_h80) (pt -3.198, -4.821) (rotation 90))
			(pad (padNum 6) (padStyleRef c130_h80) (pt -1.428, -5.424) (rotation 90))
			(pad (padNum 7) (padStyleRef c130_h80) (pt 0.342, -4.821) (rotation 90))
			(pad (padNum 8) (padStyleRef c130_h80) (pt 1.376, -3.262) (rotation 90))
			(pad (padNum 9) (padStyleRef c130_h80) (pt 1.244, -1.397) (rotation 90))
			(pad (padNum 10) (padStyleRef c130_h80) (pt -1.428, -1.424) (rotation 90))
			(pad (padNum 11) (padStyleRef c130_h80) (pt -2.381, -3.074) (rotation 90))
			(pad (padNum 12) (padStyleRef c130_h80) (pt -0.475, -3.074) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt -1.428, -2.524) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -11.428 8.476) (pt 8.572 8.476) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 8.572 8.476) (pt 8.572 -13.524) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt 8.572 -13.524) (pt -11.428 -13.524) (width 0.1))
		)
		(layerContents (layerNumRef 30)
			(line (pt -11.428 -13.524) (pt -11.428 8.476) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -7.428 -2.524) (pt -7.428 -2.524) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(arc (pt -1.428, -2.524) (radius 6) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt 4.572 -2.524) (pt 4.572 -2.524) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(arc (pt -1.428, -2.524) (radius 6) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt -10.428 2.976) (pt -10.428 -8.024) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt -10.428 -8.024) (pt -1.428 -12.524) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.428 -12.524) (pt 7.572 -8.024) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.572 -8.024) (pt 7.572 2.976) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt 7.572 2.976) (pt -1.428 7.476) (width 0.2))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.428 7.476) (pt -10.428 2.976) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -10.428 2.976) (pt -10.428 -8.024) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -10.428 -8.024) (pt -1.428 -12.524) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.428 -12.524) (pt 7.572 -8.024) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.572 -8.024) (pt 7.572 2.976) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 7.572 2.976) (pt -1.428 7.476) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.428 7.476) (pt -10.428 2.976) (width 0.1))
		)
	)
	(symbolDef "T4141012121-000" (originalName "T4141012121-000")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 7) (pt 900 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 8) (pt 900 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 9) (pt 900 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 10) (pt 900 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 11) (pt 900 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 12) (pt 900 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 670 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 700 mils 100 mils) (width 6 mils))
		(line (pt 700 mils 100 mils) (pt 700 mils -600 mils) (width 6 mils))
		(line (pt 700 mils -600 mils) (pt 200 mils -600 mils) (width 6 mils))
		(line (pt 200 mils -600 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 750 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "T4141012121-000" (originalName "T4141012121-000") (compHeader (numPins 12) (numParts 1) (refDesPrefix J)
		)
		(compPin "1" (pinName "1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "2") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "3") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "4") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "5" (pinName "5") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "6" (pinName "6") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "7" (pinName "7") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "8" (pinName "8") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "9" (pinName "9") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "10" (pinName "10") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "11" (pinName "11") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "12" (pinName "12") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "T4141012121-000"))
		(attachedPattern (patternNum 1) (patternName "T4140012121000")
			(numPads 12)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
			)
		)
		(attr "element14 Part Number" "")
		(attr "element14 Price/Stock" "")
		(attr "Manufacturer_Name" "TE Connectivity")
		(attr "Manufacturer_Part_Number" "T4141012121-000")
		(attr "Description" "Body Features: Environmental Protection IP67 | Shell Plating Material Nickel | Shell Base Material Brass | Circular Connector Insulation Material Type Polyamide 66 GF25 | O-Ring Material Silicone | Peripheral Seal Material Silicone | Environmental Protection Type Elastomer Sealed | Configuration Features: Contacts Preloaded Yes | Keying A | Number of Positions 12 | Number of Power Positions 0 | Keying & Polarized Position Locations A | Number of Signal Positions 12 | Factory Installed Backshell No | Contact")
		(attr "Datasheet Link" "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1-773955-5&DocType=Data%20Sheet&DocLang=English&PartCntxt=T4141012121-000&DocFormat=pdf")
		(attr "Height" "20 mm")
	)

)