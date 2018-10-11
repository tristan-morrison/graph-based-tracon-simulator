#include <QString>
#include <QTextStream>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#ifndef CSVRowQFile_H
#define CSVRowQFile_H

/* -----------------------------------------------
  A class for reading from a .csv file

  This class is due to Martin York on Stack Overflow
  https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
-------------------------------------------------*/

class CSVRowQFile
{
    public:
//        CSVRowQFile();
        QString const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(QTextStream& str)
        {
            QString         line;
            str.readLineInto(&line);

            QTextStream   lineStream(&line);
            QString         cell;

            m_data.clear();
            while(!lineStream.atEnd())
            {
                lineStream.readLineInto(&cell);
                QStringList stringList = cell.split(',');
                for (int k = 0; k < stringList.length(); k++) {
                    m_data.push_back(stringList[k]);
                }
            }
            // This checks for a trailing comma with no data after it.
            if (lineStream.atEnd() && cell.isEmpty())
            {
                // If there was a trailing comma then add an empty element.
                m_data.push_back("");
            }
        }
    private:
        std::vector<QString>    m_data;
};

QTextStream& operator>>(QTextStream& str, CSVRowQFile& data)
{
    data.readNextRow(str);
    return str;
}

#endif
