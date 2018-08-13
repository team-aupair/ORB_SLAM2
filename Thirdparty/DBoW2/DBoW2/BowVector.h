/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

#include <iostream>
#include <map>
#include <vector>

namespace DBoW2 {

/// Id of words
typedef unsigned int WordId;

/// Value of a word
typedef double WordValue;

/// Id of nodes in the vocabulary treee
typedef unsigned int NodeId;

/// L-norms for normalization
enum LNorm
{
  L1,
};

/// Weighting type
enum WeightingType
{
  TF_IDF,
};

/// Scoring type
enum ScoringType
{
  L1_NORM,
};

/// Vector of words to represent images
class BowVector: 
	public std::map<WordId, WordValue>
{
public:
    typedef std::map<WordId, WordValue> super;
	/** 
	 * Constructor
	 */
	BowVector(void);

	/**
	 * Destructor
	 */
	~BowVector(void);
	
	/**
	 * Adds a value to a word value existing in the vector, or creates a new
	 * word with the given value
	 * @param id word id to look for
	 * @param v value to create the word with, or to add to existing word
	 */
	void addWeight(WordId id, WordValue v);
	
	/**
	 * Adds a word with a value to the vector only if this does not exist yet
	 * @param id word id to look for
	 * @param v value to give to the word if this does not exist
	 */
	void addIfNotExist(WordId id, WordValue v);

	/**
	 * L1-Normalizes the values in the vector 
	 * @param norm_type norm used
	 */
	void normalize(LNorm norm_type);
	
	/**
	 * Prints the content of the bow vector
	 * @param out stream
	 * @param v
	 */
	friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
};

} // namespace DBoW2

#endif
