#ifndef EK305SD_h
#define EK305SD_h

#include <SD.h>

class EK305SD
{
public:
    EK305SD()
    {
    }

    int Init(int CS_PIN)
    {
        pinMode(CS_PIN, OUTPUT);
        return SD.begin(CS_PIN);
    }

    int Remove(String fileName)
    {
        if (SD.exists(fileName))
        {
            SD.remove(fileName);
            if (!SD.exists(fileName))
            {
                return 1;
            }
        }
        return 0;
    }

    int Write(String fileName, String data)
    {
        File file = SD.open(fileName, FILE_WRITE);
        if (file)
        {
            file.print(data);
            file.close();
            return 1;
        }
        else
            return 0;
    }

    int Read(String fileName)
    {
        File file = SD.open(fileName, FILE_READ);
        if (file)
        {
            while (file.available())
            {
                Serial.write(file.read());
            }
            file.close();
            return 1;
        }
        else
            return 0;
    }
};

#endif
