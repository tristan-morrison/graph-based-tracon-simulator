#include <QString>
#include <QTextStream>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "CSVRow_QFile.h"

/* -----------------------------------------------
  A class for reading from a .csv file

  This class is due to Martin York on Stack Overflow
  https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
-------------------------------------------------*/


QString const& CSVRow_QFile::operator[](std::size_t index) const
{
    return m_data[index];
}

std::size_t CSVRow_QFile::size() const
{
    return m_data.size();
}

void CSVRow_QFile::readNextRow(QTextStream& str)
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
            m_data.push_back(cell);
        }
    }
    // This checks for a trailing comma with no data after it.
    if (lineStream.atEnd() && cell.isEmpty())
    {
        // If there was a trailing comma then add an empty element.
        m_data.push_back("");
    }
}

QTextStream& operator>>(QTextStream& str, CSVRow_QFile& data)
{
    data.readNextRow(str);
    return str;
}
