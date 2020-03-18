/*!
 *
 *  Spectrum Colour Manager - v1.0.0
    Copyright (c) 2018  - Richard Roberts

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (Spectrum), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

*/

#ifndef COLOURMANAGER
#define COLOURMANAGER
#include <iostream>
#include <sstream>
#include <vector>
#include <time.h>
#include <algorithm>

namespace azure
{
    namespace color
    {
        /** The CMClassification enum.
     *  These enums represent the type of colour map through their classification.
     */
        enum class CMClassification {
            SEQUENTIAL,   /**< Represents the sequential colour map - one colour changing in hue. */
            DIVERGING,    /**< Represents the diverging colour map - two colours diverging from a neutral colour. */
            QUALITATIVE,  /**< Represents the qualitative colour map - many differing colours for discrete data. */
            ANY           /**< Represents any type of colour map.*/
        };

        /*! The Colour class - This holds the colour channel values and functions needed to produce RGB (float or int value) or hex values */
        class Colour {
        public:

            //! An empty constructor for the class. */
            Colour() = default;

            //! A constructor for the Colour class.
                /*!
                  \param R The red colour channel - Either 0-1 OR 0-255
                  \param G The green colour channel - Either 0-1 OR 0-255
                  \param B The blue colour channel - Either 0-1 OR 0-255
                  \param A The alpha colour channel - Either 0-1 OR 0-255
                */
            Colour(float R, float G, float B, float A) {
                if (R > 1.0f || G > 1.0f || B > 1.0f) {
                    float rVal = (float)R / (float)255.0f;
                    float gVal = (float)G / (float)255.0f;
                    float bVal = (float)B / (float)255.0f;
                    setR(rVal);
                    setG(gVal);
                    setB(bVal);
                    setAlpha(A);
                }
                else {
                    setR(R);
                    setG(G);
                    setB(B);
                    setAlpha(A);
                }

            }

            //! A constructor for the Colour class.
                /*!
                  \param R The red colour channel - float value either 0-1 OR 0-255
                  \param G The green colour channel - float value Either 0-1 OR 0-255
                  \param B The blue colour channel - float value Either 0-1 OR 0-255
                  \param A The alpha colour channel - float value Either 0-1 OR 0-255
                  \param A The alpha colour channel - float value Either 0-1 OR 0-255
                */
            Colour(float R, float G, float B, float A, std::string name) {
                setNameID(name);
                if (R > 1.0f || G > 1.0f || B > 1.0f) {
                    float rVal = (float)R / (float)255.0f;
                    float gVal = (float)G / (float)255.0f;
                    float bVal = (float)B / (float)255.0f;
                    setR(rVal);
                    setG(gVal);
                    setB(bVal);
                    setAlpha(A);
                }
                else {
                    setR(R);
                    setG(G);
                    setB(B);
                    setAlpha(A);
                }

            }

            //! A constructor for the Colour class created using a colour hex value
                /*!
                  \param hex - a string that contains the hex colour value.
                */
            Colour(std::string hex) {

                std::string r = hex.substr(1, 2);
                std::string g = hex.substr(3, 2);
                std::string b = hex.substr(5, 2);

                int newR, newG, newB;
                std::stringstream sa;
                sa << std::hex << r;
                sa >> newR;

                std::stringstream sb;
                sb << std::hex << g;
                sb >> newG;

                std::stringstream sc;
                sc << std::hex << b;
                sc >> newB;


                float rVal = (float)newR / (float)255.0f;
                float gVal = (float)newG / (float)255.0f;
                float bVal = (float)newB / (float)255.0f;

                setR(rVal);
                setG(gVal);
                setB(bVal);
            }

            //! Returns the hex value of the colour object as a std::string.
            /*!
              \return std::string hex value
            */
            inline std::string getHexColour() {
                std::stringstream stream;
                stream << std::hex << (int)(getR() * 255.0f) << (int)(getG() * 255.0f) << (int)(getB() * 255.0f);
                std::string result(stream.str());
                return result;
            }

            //! Sets the red channel value.
            /*!
              \param float R - the red channel value.
            */
            inline void setR(float R) {
                m_R = R;
            }

            //! Sets the green channel value.
            /*!
              \param float G - the green channel value.
            */
            inline void setG(float G) {
                m_G = G;
            }

            //! Sets the blue channel value.
            /*!
              \param float B - the blue channel value.
            */
            inline void setB(float B) {
                m_B = B;
            }

            //! Sets the colour alpha value.
            /*!
              \param float Alpha - the alpha value.
            */
            inline void setAlpha(float Alpha) {
                m_Alpha = Alpha;
            }

            //! Sets the colour name
            /*!
              \param std::string name - the colour name.
            */
            inline void setNameID(std::string name) {
                m_NameID = name;
            }


            //! Gets the red channel value (0-255 int)
            /*!
              \return int R channel value.
            */
            inline int getIntR() {
                return m_R * 255.0f;
            }

            //! Gets the green channel value (0-255 int)
            /*!
              \return int B channel value.
            */
            inline int getIntG() {
                return m_G * 255.0f;
            }

            //! Gets the blue channel value (0-255 int)
            /*!
              \return int B channel value.
            */
            inline int getIntB() {
                return m_B * 255.0f;
            }

            //! Gets the red channel value (0-1 float)
            /*!
              \return float R channel value.
            */
            inline float getR() {
                return m_R;
            }

            //! Gets the green channel value (0-1 float)
            /*!
              \return float G channel value.
            */
            inline float getG() {
                return m_G;
            }

            //! Gets the blue channel value (0-1 float)
            /*!
              \return float B channel value.
            */
            inline float getB() {
                return m_B;
            }

            //! Gets the alpha channel value (0-1 float)
            /*!
              \return float alpha channel value.
            */
            inline float getAlpha() {
                return m_Alpha;
            }

            //! Gets the colour name
            /*!
              \return std::string m_NameID name value;
            */
            inline std::string getNameID() {
                return m_NameID;
            }

        private:
            float m_R;
            float m_G;
            float m_B;
            float m_Alpha;

            std::string m_NameID;
        };

        /*! This class holds a vector a colours which make up the colours of the colour map */
        class ColourMap {
        public:

            ColourMap(std::string name = "") 
            {
                getMapName() = name;
            }

            inline void addColour(Colour col) {
                getColourList().push_back(col);
            }
            inline void addColour(float R, float G, float B, float F) {
                Colour col(R, G, B, F);
                addColour(col);
            }
            inline void addColour(float R, float G, float B, float F, std::string name) {
                Colour col(R, G, B, F);
                col.setNameID(name);
                addColour(col);
            }

            inline Colour& operator[](const int i) {
                return m_ColourList[i];
            }

            inline Colour& operator[](const std::string name) {
                for (int i = 0; i < m_ColourList.size(); i++) {
                    if (m_ColourList[i].getNameID() == name) {
                        return m_ColourList[i];
                    }
                }

                return Colour(0.0f, 0.0f, 0.0f, 1.0f, "NoColour");
            }

            inline const Colour& operator[](const int i)const {
                return m_ColourList[i];
            }

            inline std::vector<Colour>& getColourList() {
                return m_ColourList;
            }

            inline const std::vector<Colour>& getColourList()const {
                return m_ColourList;
            }

            inline std::string getMapName() {
                return m_MapName;
            }

            inline int classCount() {
                m_ColourList.size();
            }

            inline void setMapName(std::string name) {
                m_MapName = name;
            }

            inline CMClassification& getClassification() {
                return m_Cls;
            }

            inline void setClassification(CMClassification cls) {
                m_Cls = cls;
            }

            inline int getIndex() {
                return m_Index;
            }

            inline void setIndex(int index) {
                m_Index = index;
            }

        private:
            std::vector<Colour> m_ColourList;
            std::string m_MapName;
            CMClassification m_Cls;
            int m_Index;
        };

        /*! The CMList holds a list of colour maps which the user can select from to set the currently used  colour map. */
        class CMList {
        public:

            inline static std::vector<ColourMap> getMapList(CMClassification cls = CMClassification::ANY) {
                if (cls == CMClassification::ANY) {
                    return MapList();
                }

                std::vector<ColourMap> filtered;
                std::copy_if(MapList().begin(), MapList().end(), std::back_inserter(filtered), [&](ColourMap& cm) {
                    return cls == cm.getClassification();
                    });

                return filtered;
            }

            inline static void addColourMap(ColourMap map) {
                MapList().push_back(map);
            }

            inline static std::vector<ColourMap>& returnCompleteMapList() {
                return MapList();
            }

            inline static void setupIndexesInList() {
                for (int index = 0; index < MapList().size(); index++) {
                    MapList()[index].setIndex(index);
                }
            }

        private:
            static std::vector<ColourMap>& MapList() { static std::vector<ColourMap> m_MapList; return m_MapList; }
        };

        /*! The ColourManager class is the main manager for the colour maps. It handles the calculations of colour values and the current colour map.*/
        class ColourManager {
        public:

            static int& ColourMapIndex() { static int CMI{}; return CMI; }
            static void setColourMapIndex(int index) {
                ColourMapIndex() = index;
            }
            static bool& InvertColourMapFlag() { static bool ICM{ false }; return ICM; }

            static void InvertColourMap() {
                InvertColourMapFlag() = !InvertColourMapFlag();
            }

            inline float getUpperRange() {
                return m_UpperRange;
            }

            inline void setUpperRange(float UpperRange) {
                m_UpperRange = UpperRange;
            }

            inline float getLowerRange() {
                return m_LowerRange;
            }

            inline void setLowerRange(float LowerRange) {
                m_LowerRange = LowerRange;
            }

            inline ColourMap& getCurrentColourMap() {
                return CurrentColourMap();
            }

            static inline void setCurrentColourMap(ColourMap& map) {
                ColourMapIndex() = map.getIndex();
                CurrentColourMap() = map;
            }

            inline CMList& getCMList() {
                return ColourMapList();
            }

            static void Init_ColourManager() {

                ColourMap redGreen("Red->Green");
                redGreen.setClassification(CMClassification::DIVERGING);
                redGreen.addColour(1.0f, 0.0f, 0.0f, 1.0f);
                redGreen.addColour(0.0f, 1.0f, 0.0f, 1.0f);
                ColourMapList().addColourMap(redGreen);

                ColourMap cbFriendly;
                cbFriendly.setMapName("Colour Blind Friendly");
                cbFriendly.setClassification(CMClassification::DIVERGING);
                cbFriendly.addColour(103.0f, 0.0f, 31.0f, 1);
                cbFriendly.addColour(178.0f, 24.0f, 43.0f, 1);
                cbFriendly.addColour(214.0f, 96.0f, 77.0f, 1);
                cbFriendly.addColour(244.0f, 165.0f, 130.0f, 1);
                cbFriendly.addColour(253.0f, 219.0f, 199.0f, 1);
                cbFriendly.addColour(247.0f, 247.0f, 247.0f, 1);
                cbFriendly.addColour(209.0f, 229.0f, 240.0f, 1);
                cbFriendly.addColour(146.0f, 197.0f, 222.0f, 1);
                cbFriendly.addColour(67.0f, 147.0f, 195.0f, 1);
                cbFriendly.addColour(33.0f, 102.0f, 172.0f, 1);
                cbFriendly.addColour(5.0f, 48.0f, 97.0f, 1);
                ColourMapList().addColourMap(cbFriendly);

                ColourMap cbGreenRed;
                cbGreenRed.setMapName("Red Green Colour Brewer");
                cbGreenRed.setClassification(CMClassification::DIVERGING);
                cbGreenRed.addColour(165.0f, 0.0f, 38.0f, 1.0f);
                cbGreenRed.addColour(215.0f, 48.0f, 39.0f, 1.0f);
                cbGreenRed.addColour(244.0f, 109.0f, 67.0f, 1.0f);
                cbGreenRed.addColour(253.0f, 174.0f, 97.0f, 1.0f);
                cbGreenRed.addColour(254.0f, 224.0f, 139.0f, 1.0f);
                cbGreenRed.addColour(255.0f, 255.0f, 191.0f, 1.0f);
                cbGreenRed.addColour(217.0f, 239.0f, 139.0f, 1.0f);
                cbGreenRed.addColour(166.0f, 217.0f, 106.0f, 1.0f);
                cbGreenRed.addColour(102.0f, 189.0f, 99.0f, 1.0f);
                cbGreenRed.addColour(26.0f, 152.0f, 80.0f, 1.0f);
                cbGreenRed.addColour(0.0f, 104.0f, 55.0f, 1.0f);
                ColourMapList().addColourMap(cbGreenRed);

                ColourMap redSeq;
                redSeq.setMapName("Red Sequential");
                redSeq.setClassification(CMClassification::SEQUENTIAL);
                redSeq.addColour(255, 255, 204, 1.0f);
                redSeq.addColour(255, 237, 160, 1.0f);
                redSeq.addColour(254, 217, 118, 1.0f);
                redSeq.addColour(254, 178, 76, 1.0f);
                redSeq.addColour(253, 141, 60, 1.0f);
                redSeq.addColour(252, 78, 42, 1.0f);
                redSeq.addColour(227, 26, 28, 1.0f);
                redSeq.addColour(189, 0, 38, 1.0f);
                redSeq.addColour(128, 0, 38, 1.0f);
                ColourMapList().addColourMap(redSeq);

                ColourMap redBlue;
                redBlue.setMapName("Colour Brewer, Red -> Blue");
                redBlue.getClassification() = CMClassification::DIVERGING;
                redBlue.addColour(165, 0, 38, 1.0f);
                redBlue.addColour(215, 48, 39, 1.0f);
                redBlue.addColour(244, 109, 67, 1.0f);
                redBlue.addColour(253, 174, 97, 1.0f);
                redBlue.addColour(254, 224, 144, 1.0f);
                redBlue.addColour(255, 255, 191, 1.0f);
                redBlue.addColour(224, 243, 248, 1.0f);
                redBlue.addColour(171, 217, 233, 1.0f);
                redBlue.addColour(116, 173, 209, 1.0f);
                redBlue.addColour(69, 117, 180, 1.0f);
                redBlue.addColour(49, 54, 149, 1.0f);
                ColourMapList().addColourMap(redBlue);

                ColourMap qualitative1;
                qualitative1.setMapName("Qualitative CB");
                qualitative1.setClassification(CMClassification::QUALITATIVE);
                qualitative1.addColour(166, 206, 227, 1.0f);
                qualitative1.addColour(31, 120, 180, 1.0f);
                qualitative1.addColour(178, 223, 138, 1.0f);
                qualitative1.addColour(51, 160, 44, 1.0f);
                qualitative1.addColour(251, 154, 153, 1.0f);
                qualitative1.addColour(227, 26, 28, 1.0f);
                qualitative1.addColour(253, 191, 111, 1.0f);
                qualitative1.addColour(255, 127, 0, 1.0f);
                qualitative1.addColour(202, 178, 214, 1.0f);
                qualitative1.addColour(106, 61, 154, 1.0f);
                qualitative1.addColour(255, 255, 153, 1.0f);
                qualitative1.addColour(177, 89, 40, 1.0f);
                ColourMapList().addColourMap(qualitative1);

                ColourMap qualitative2;
                qualitative2.setMapName("Qualitative CB2 with names");
                qualitative2.setClassification(CMClassification::QUALITATIVE);
                qualitative2.addColour(141, 211, 199, 1.0f, "Col1");
                qualitative2.addColour(255, 255, 179, 1.0f, "Col2");
                qualitative2.addColour(190, 186, 218, 1.0f, "Col3");
                qualitative2.addColour(251, 128, 114, 1.0f, "Col4");
                qualitative2.addColour(128, 177, 211, 1.0f, "Col5");
                qualitative2.addColour(253, 180, 98, 1.0f, "Col6");
                qualitative2.addColour(179, 222, 105, 1.0f, "Col7");
                qualitative2.addColour(252, 205, 229, 1.0f, "Col8");
                qualitative2.addColour(217, 217, 217, 1.0f, "Col9");
                qualitative2.addColour(188, 128, 189, 1.0f, "Col10");
                qualitative2.addColour(204, 235, 197, 1.0f, "Col11");
                qualitative2.addColour(255, 237, 111, 1.0f, "Col12");
                ColourMapList().addColourMap(qualitative2);

                //Add indexes to the colour maps
                ColourMapList().setupIndexesInList();

            }

            inline static void setCurrentColourMap() {
                CurrentColourMap() = ColourMapList().getMapList()[ColourMapIndex()];
                checkInvertedColours();
            }

            inline static ColourMap returnRandomColourMap(int seed = -1, int listSize = 10) {
                ColourMap list;
                list.setClassification(CMClassification::QUALITATIVE);
                if (seed == -1) {
                    std::srand(time(NULL));
                }
                else {
                    std::srand(seed);
                }

                for (int i = 0; i < listSize; i++) {
                    float R = (float)std::rand() / (float)RAND_MAX;
                    float G = (float)std::rand() / (float)RAND_MAX;
                    float B = (float)std::rand() / (float)RAND_MAX;
                    Colour c(R, G, B, 1.0f);
                    list.getColourList().push_back(c);
                }

                return list;
            }

            inline Colour getInterpolatedColour(float value) {

                //This ceils or floors any values outside of the range to prevent componding errors.
                if (value <= getLowerRange()) { value = getLowerRange(); }
                if (value >= getUpperRange()) { value = getUpperRange(); }
                Colour colourOutput;
                float distance = abs((float)getUpperRange() - (float)getLowerRange());
                float valueAlongRange = (float)abs(value - getLowerRange()) / (float)abs(distance);
                if (valueAlongRange < 0) {
                    return colourOutput;
                }
                else if (valueAlongRange > 1.0f) {
                    valueAlongRange = 1.0f;
                }

                int size = CurrentColourMap().getColourList().size();
                float ratioSegment = (float)1.0f / (float)(size - 1.0f);
                float ratioThroughSegment;
                for (int i = 1; i < size; i++) {
                    float lowerSegment = (float)(i - 1) * (float)ratioSegment;
                    float upperSegemnt = (float)(i) * (float)ratioSegment;

                    if (valueAlongRange >= lowerSegment && valueAlongRange <= upperSegemnt) {

                        ratioThroughSegment = ((float)(valueAlongRange - lowerSegment) / (float)(upperSegemnt - lowerSegment));

                        Colour colour1 = CurrentColourMap().getColourList()[i - 1];
                        Colour colour2 = CurrentColourMap().getColourList()[i];

                        //R Value
                        //This calculates the R value for the output colour;
                        if (colour1.getR() < colour2.getR()) {
                            colourOutput.setR((float)colour1.getR() + (float)((colour2.getR() - colour1.getR()) * (float)ratioThroughSegment));
                        }
                        else if (colour2.getR() < colour1.getR()) {
                            colourOutput.setR((float)colour1.getR() - (float)((colour1.getR() - colour2.getR()) * (float)ratioThroughSegment));
                        }
                        else {
                            colourOutput.setR((float)colour1.getR());
                        }

                        //G Value
                        if (colour1.getG() < colour2.getG()) {
                            colourOutput.setG(colour1.getG() + ((float)(colour2.getG() - colour1.getG()) * (float)ratioThroughSegment));
                        }
                        else if (colour2.getG() < colour1.getG()) {
                            colourOutput.setG((float)colour1.getG() - ((float)(colour1.getG() - colour2.getG()) * (float)ratioThroughSegment));
                        }
                        else {
                            colourOutput.setG((float)colour1.getG());
                        }

                        //B Value
                        if (colour1.getB() < colour2.getB()) {
                            colourOutput.setB(colour1.getB() + ((float)(colour2.getB() - colour1.getB()) * (float)ratioThroughSegment));
                        }
                        else if (colour2.getB() < colour1.getB()) {
                            colourOutput.setB((float)colour1.getB() - ((float)(colour1.getB() - colour2.getB()) * (float)ratioThroughSegment));
                        }
                        else {
                            colourOutput.setB((float)colour1.getB());
                        }

                        //Alpha Value
                        if (colour1.getAlpha() < colour2.getAlpha()) {
                            colourOutput.setAlpha((float)colour1.getAlpha() + ((float)(colour2.getAlpha() - colour1.getAlpha()) * (float)ratioThroughSegment));
                        }
                        else if (colour2.getAlpha() < colour1.getAlpha()) {
                            colourOutput.setAlpha((float)colour1.getAlpha() - ((float)(colour1.getAlpha() - colour2.getAlpha()) * (float)ratioThroughSegment));
                        }
                        else {
                            colourOutput.setAlpha((float)colour1.getAlpha());
                        }

                        return colourOutput;

                    }
                }

                return colourOutput;

            }

            inline Colour getClassColour(float value) {
                //This ceils or floors any values outside of the range to prevent componding errors.
                if (value <= getLowerRange()) { value = getLowerRange(); }
                if (value >= getUpperRange()) { value = getUpperRange(); }
                Colour colourOutput(0, 0, 0, 1);
                float distance = abs((float)getUpperRange() - (float)getLowerRange());
                float valueAlongRange = (float)abs(value - getLowerRange()) / (float)abs(distance);
                if (valueAlongRange < 0) {
                    return colourOutput;
                }
                else if (valueAlongRange > 1.0f) {
                    valueAlongRange = 1.0f;
                }

                int size = CurrentColourMap().getColourList().size();
                //This offset prevents any 1.0f values flooring to a larger index than the list size.
                valueAlongRange -= (0.0005);
                int index = (int)floor(valueAlongRange * size);

                if (index < CurrentColourMap().getColourList().size()) {
                    colourOutput = CurrentColourMap()[index];
                };

                return colourOutput;
            }

            inline Colour getColourFromSeed(unsigned int seed) {
                std::srand(seed);
                float R = (float)std::rand() / (float)RAND_MAX;
                float G = (float)std::rand() / (float)RAND_MAX;
                float B = (float)std::rand() / (float)RAND_MAX;
                Colour c(R, G, B, 1.0f);
                return c;
            }

            inline Colour getColourFromIndex(int index) {
                try {
                    if (index < CurrentColourMap().getColourList().size() && index >= 0) {
                        return CurrentColourMap()[index];
                    }
                    else {
                        throw 0;
                    }
                }
                catch (int e) {
                    std::cout << "Index out of range in getColourFromIndex() " << e;
                }
            }

            inline Colour getColourFromName(std::string name) {
                try {
                    for (int i = 0; i < CurrentColourMap().getColourList().size(); i++) {
                        if (CurrentColourMap()[i].getNameID() == name) {
                            return CurrentColourMap()[i];
                        }
                    }
                    throw 1;
                }
                catch (std::exception e) {
                    return Colour(0, 0, 0, 1);
                    std::cout << "Colour Name not found in getColourFromName() ";
                }
            }

            ColourManager(float lowerValue, float upperValue) {
                setUpperRange(upperValue);
                setLowerRange(lowerValue);
                setCurrentColourMap();
            }

        private:

            static ColourMap& CurrentColourMap() { static ColourMap m_ColourList; return m_ColourList; }
            static CMList& ColourMapList() { static CMList m_ColourMapList; return m_ColourMapList; }

            float m_UpperRange;
            float m_LowerRange;

            inline static void checkInvertedColours() {
                if (InvertColourMapFlag()) {
                    std::reverse(CurrentColourMap().getColourList().begin(), CurrentColourMap().getColourList().end());
                }
            }
        };
    }
}


#endif // COLOURMANAGER
