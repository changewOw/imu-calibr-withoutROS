#ifndef ALLAN_H
#define ALLAN_H

#include "type.h"
#include <iostream>
#include <math.h>
#include <vector>

namespace imu
{

class AllanGyr
{
    public:
    AllanGyr( std::string name, int maxCluster = 10000 );
    ~AllanGyr( );
    void pushRadPerSec( double data, double time );
    void pushDegreePerSec( double data, double time );
    void pushDegreePerHou( double data, double time );
    void calc( );

    std::vector< double > getVariance( ) const;
    std::vector< double > getDeviation( );
    std::vector< double > getTimes( );
    std::vector< int > getFactors( ) const;
    double getAvgValue( );
    double getFreq( ) const;

    private:
    std::vector< double > calcVariance( double period );

    std::vector< double > calcThetas( const double freq );
    void initStrides( );
    std::vector< double > getLogSpace( float a, float b );
    double getAvgFreq( ) { return 1.0 / getAvgDt( ); }
    double getAvgDt( );
    double getAvgPeriod( ) { return getAvgDt( ); }
    int getFactorsNum( ) { return numFactors; }

    std::string m_name;
    double m_freq;
    int numData;
    std::vector< GyrData > m_rawData;
    std::vector< double > m_thetas;
    int numCluster;
    int numFactors;
    std::vector< int > mFactors;

    std::vector< double > mVariance;
};
}

#endif // ALLAN_H
