/**
 * File: ScoringObject.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions to compute bow scores 
 * License: see the LICENSE.txt file
 *
 */

#include <cfloat>
#include "TemplatedVocabulary.h"
#include "BowVector.h"

using namespace DBoW2;

// If you change the type of WordValue, make sure you change also the
// epsilon value (this is needed by the KL method)
const double GeneralScoring::LOG_EPS = log(DBL_EPSILON); // FLT_EPSILON

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

double L1Scoring::score(const BowVector &v1, const BowVector &v2) const
{
    BowVector::const_iterator v1_it, v2_it;
    const BowVector::const_iterator v1_end = v1.end();
    const BowVector::const_iterator v2_end = v2.end();
    
    v1_it = v1.begin();
    v2_it = v2.begin();
    
    double score = 0;
    
    while(v1_it != v1_end && v2_it != v2_end)
    {
        const WordValue& vi = v1_it->second;
        const WordValue& wi = v2_it->second;
        
        if(v1_it->first == v2_it->first)
        {
            score += fmin(vi, wi);
            
            // move v1 and v2 forward
            ++v1_it;
            ++v2_it;
        }
        else if(v1_it->first < v2_it->first)
        {
            // move v1 forward
            v1_it = v1.lower_bound(v2_it->first);
            // v1_it = (first element >= v2_it.id)
        }
        else
        {
            // move v2 forward
            v2_it = v2.lower_bound(v1_it->first);
            // v2_it = (first element >= v1_it.id)
        }
    }
    
    // ||v - w||_{L1} = 2 + Sum(|v_i - w_i| - |v_i| - |w_i|) 
    //		for all i | v_i != 0 and w_i != 0 
    // (Nister, 2006)
    // scaled_||v - w||_{L1} = 1 - 0.5 * ||v - w||_{L1}
    score = 1 + score;

    return score; // [0..1]
}
