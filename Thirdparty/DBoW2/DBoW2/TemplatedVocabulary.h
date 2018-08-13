/**
 * This is a modified version of TemplatedVocabulary.h from DBoW2 (see below).
 * Added functions: Save and Load from text/binary files without using cv::FileStorage.
 * Date: August 2015
 * Raúl Mur-Artal
 */

/**
 * File: TemplatedVocabulary.h
 * Date: February 2011
 * Author: Dorian Galvez-Lopez
 * Description: templated vocabulary
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_TEMPLATED_VOCABULARY__
#define __D_T_TEMPLATED_VOCABULARY__

#include <cassert>

#include <vector>
#include <numeric>
#include <fstream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <limits>

#include "FeatureVector.h"
#include "BowVector.h"
#include "ScoringObject.h"

#include "../DUtils/Random.h"

#include <iostream>

using namespace std;

namespace DBoW2 {

/// @param TDescriptor class of descriptor
/// @param F class of descriptor functions
template<class TDescriptor, class F>
/// Generic Vocabulary
class TemplatedVocabulary
{
public:

  /**
   * Initiates an empty vocabulary
   * @param k branching factor
   * @param L depth levels
   * @param weighting weighting type
   * @param scoring scoring type
   */
  TemplatedVocabulary(int k = 10, int L = 5,
      WeightingType weighting = TF_IDF, ScoringType scoring = L1_NORM);

  /**
   * Destructor
   */
  virtual ~TemplatedVocabulary();

  /**
   * Assigns the given vocabulary to this by copying its data and removing
   * all the data contained by this vocabulary before
   * @param voc
   * @return reference to this vocabulary
   */
  TemplatedVocabulary<TDescriptor, F>& operator=(
      const TemplatedVocabulary<TDescriptor, F> &voc);

  /**
   * Returns the number of words in the vocabulary
   * @return number of words
   */
  virtual inline unsigned int size() const;

  /**
   * Returns whether the vocabulary is empty (i.e. it has not been trained)
   * @return true iff the vocabulary is empty
   */
  virtual inline bool empty() const;

  /**
   * Transforms a set of descriptores into a bow vector
   * @param features
   * @param v (out) bow vector of weighted words
   */
  virtual void transform(const std::vector<TDescriptor>& features, BowVector &v)
      const;

  /**
   * Transform a set of descriptors into a bow vector and a feature vector
   * @param features
   * @param v (out) bow vector
   * @param fv (out) feature vector of nodes and feature indexes
   * @param levelsup levels to go up the vocabulary tree to get the node index
   */
  virtual void transform(const std::vector<TDescriptor>& features,
    BowVector &v, FeatureVector &fv, int levelsup) const;

  /**
   * Transforms a single feature into a word (without weight)
   * @param feature
   * @return word id
   */
  virtual WordId transform(const TDescriptor& feature) const;

  /**
   * Returns the score of two vectors
   * @param a vector
   * @param b vector
   * @return score between vectors
   * @note the vectors must be already sorted and normalized if necessary
   */
  inline double score(const BowVector &a, const BowVector &b) const;

  /**
   * Returns the id of the node that is "levelsup" levels from the word given
   * @param wid word id
   * @param levelsup 0..L
   * @return node id. if levelsup is 0, returns the node id associated to the
   *   word id
   */
  virtual NodeId getParentNode(WordId wid, int levelsup) const;

  /**
   * Returns the ids of all the words that are under the given node id,
   * by traversing any of the branches that goes down from the node
   * @param nid starting node id
   * @param words ids of words
   */
  void getWordsFromNode(NodeId nid, std::vector<WordId> &words) const;

  /**
   * Returns the branching factor of the tree (k)
   * @return k
   */
  inline int getBranchingFactor() const { return m_k; }

  /**
   * Returns the depth levels of the tree (L)
   * @return L
   */
  inline int getDepthLevels() const { return m_L; }

  /**
   * Returns the real depth levels of the tree on average
   * @return average of depth levels of leaves
   */
  float getEffectiveLevels() const;

  /**
   * Returns the descriptor of a word
   * @param wid word id
   * @return descriptor
   */
  virtual inline TDescriptor getWord(WordId wid) const;

  /**
   * Returns the weight of a word
   * @param wid word id
   * @return weight
   */
  virtual inline WordValue getWordWeight(WordId wid) const;

  /**
   * Returns the weighting method
   * @return weighting method
   */
  inline WeightingType getWeightingType() const { return m_weighting; }

  /**
   * Returns the scoring method
   * @return scoring method
   */
  inline ScoringType getScoringType() const { return m_scoring; }

  /**
   * Changes the weighting method
   * @param type new weighting type
   */
  inline void setWeightingType(WeightingType type);

  /**
   * Loads the vocabulary from a binary file
   * @param filename
   */
  bool loadFromBinaryFile(const std::string &filename);

  /**
   * Stops those words whose weight is below minWeight.
   * Words are stopped by setting their weight to 0. There are not returned
   * later when transforming image features into vectors.
   * Note that when using IDF or TF_IDF, the weight is the idf part, which
   * is equivalent to -log(f), where f is the frequency of the word
   * (f = Ni/N, Ni: number of training images where the word is present,
   * N: number of training images).
   * Note that the old weight is forgotten, and subsequent calls to this
   * function with a lower minWeight have no effect.
   * @return number of words stopped now
   */
  virtual int stopWords(double minWeight);

protected:

  /// Pointer to descriptor
  typedef const TDescriptor *pDescriptor;

  /// Tree node
  struct Node
  {
    /// Node id
    NodeId id;
    /// Weight if the node is a word
    WordValue weight;
    /// Children
    vector<NodeId> children;
    /// Parent node (undefined in case of root)
    NodeId parent;
    /// Node descriptor
    TDescriptor descriptor;

    /// Word id if the node is a word
    WordId word_id;

    /**
     * Empty constructor
     */
    Node(): id(0), weight(0), parent(0), word_id(0){}

    /**
     * Constructor
     * @param _id node id
     */
    Node(NodeId _id): id(_id), weight(0), parent(0), word_id(0){}

    /**
     * Returns whether the node is a leaf node
     * @return true iff the node is a leaf
     */
    inline bool isLeaf() const { return children.empty(); }
  };

protected:

  /**
   * Creates an instance of the scoring object accoring to m_scoring
   */
  void createScoringObject();

  /**
   * Returns a set of pointers to descriptores
   * @param training_features all the features
   * @param features (out) pointers to the training features
   */
  void getFeatures(
    const vector<vector<TDescriptor> > &training_features,
    vector<pDescriptor> &features) const;

  /**
   * Returns the word id associated to a feature
   * @param feature
   * @param id (out) word id
   * @param weight (out) word weight
   * @param nid (out) if given, id of the node "levelsup" levels up
   * @param levelsup
   */
  virtual void transform(const TDescriptor &feature,
    WordId &id, WordValue &weight, NodeId* nid = NULL, int levelsup = 0) const;

  /**
   * Returns the word id associated to a feature
   * @param feature
   * @param id (out) word id
   */
  virtual void transform(const TDescriptor &feature, WordId &id) const;

  /**
   * Create the words of the vocabulary once the tree has been built
   */
  void createWords();

  /**
   * Sets the weights of the nodes of tree according to the given features.
   * Before calling this function, the nodes and the words must be already
   * created (by calling HKmeansStep and createWords)
   * @param features
   */
  void setNodeWeights(const vector<vector<TDescriptor> > &features);

protected:

  /// Branching factor
  int m_k;

  /// Depth levels
  int m_L;

  /// Weighting method
  WeightingType m_weighting;

  /// Scoring method
  ScoringType m_scoring;

  /// Object for computing scores
  GeneralScoring* m_scoring_object;

  /// Tree nodes
  std::vector<Node> m_nodes;

  /// Words of the vocabulary (tree leaves)
  /// this condition holds: m_words[wid]->word_id == wid
  std::vector<Node*> m_words;

};

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::TemplatedVocabulary
  (int k, int L, WeightingType weighting, ScoringType scoring)
  : m_k(k), m_L(L), m_weighting(weighting), m_scoring(scoring),
  m_scoring_object(NULL)
{
  createScoringObject();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createScoringObject()
{
  delete m_scoring_object;
  m_scoring_object = new L1Scoring;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setWeightingType(WeightingType type)
{
  this->m_weighting = type;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor,F>::~TemplatedVocabulary()
{
  delete m_scoring_object;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TemplatedVocabulary<TDescriptor, F>&
TemplatedVocabulary<TDescriptor,F>::operator=
  (const TemplatedVocabulary<TDescriptor, F> &voc)
{
  this->m_k = voc.m_k;
  this->m_L = voc.m_L;
  this->m_scoring = voc.m_scoring;
  this->m_weighting = voc.m_weighting;

  this->createScoringObject();

  this->m_nodes.clear();
  this->m_words.clear();

  this->m_nodes = voc.m_nodes;
  this->createWords();

  return *this;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getFeatures(
  const vector<vector<TDescriptor> > &training_features,
  vector<pDescriptor> &features) const
{
  features.resize(0);

  typename vector<vector<TDescriptor> >::const_iterator vvit;
  typename vector<TDescriptor>::const_iterator vit;
  for(vvit = training_features.begin(); vvit != training_features.end(); ++vvit)
  {
    features.reserve(features.size() + vvit->size());
    for(vit = vvit->begin(); vit != vvit->end(); ++vit)
    {
      features.push_back(&(*vit));
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::createWords()
{
  m_words.resize(0);

  if(!m_nodes.empty())
  {
    m_words.reserve( (int)pow((double)m_k, (double)m_L) );

    typename vector<Node>::iterator nit;

    nit = m_nodes.begin(); // ignore root
    for(++nit; nit != m_nodes.end(); ++nit)
    {
      if(nit->isLeaf())
      {
        nit->word_id = m_words.size();
        m_words.push_back( &(*nit) );
      }
    }
  }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::setNodeWeights
    (const vector<vector<TDescriptor> > &training_features)
{
    const unsigned int NWords = m_words.size();
    const unsigned int NDocs = training_features.size();

    // we calculte the idf path now

    // Note: this actually calculates the idf part of the tf-idf score.
    // The complete tf-idf score is calculated in ::transform

    vector<unsigned int> Ni(NWords, 0);
    vector<bool> counted(NWords, false);

    typename vector<vector<TDescriptor> >::const_iterator mit;
    typename vector<TDescriptor>::const_iterator fit;

    for(mit = training_features.begin(); mit != training_features.end(); ++mit)
    {
        fill(counted.begin(), counted.end(), false);

        for(fit = mit->begin(); fit < mit->end(); ++fit)
        {
            WordId word_id;
            transform(*fit, word_id);

            if(!counted[word_id])
            {
                Ni[word_id]++;
                counted[word_id] = true;
            }
        }
    }

    // set ln(N/Ni)
    for(unsigned int i = 0; i < NWords; i++)
    {
        if(Ni[i] > 0)
        {
            m_words[i]->weight = log((double)NDocs / (double)Ni[i]);
        }// else // This cannot occur if using kmeans++
    }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline unsigned int TemplatedVocabulary<TDescriptor,F>::size() const
{
    return m_words.size();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline bool TemplatedVocabulary<TDescriptor,F>::empty() const
{
    return m_words.empty();
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
float TemplatedVocabulary<TDescriptor,F>::getEffectiveLevels() const
{
    long sum = 0;
    typename std::vector<Node*>::const_iterator wit;
    for(wit = m_words.begin(); wit != m_words.end(); ++wit)
    {
        const Node *p = *wit;

        for(; p->id != 0; sum++) p = &m_nodes[p->parent];
    }

    return (float)((double)sum / (double)m_words.size());
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
TDescriptor TemplatedVocabulary<TDescriptor,F>::getWord(WordId wid) const
{
    return m_words[wid]->descriptor;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
WordValue TemplatedVocabulary<TDescriptor, F>::getWordWeight(WordId wid) const
{
    return m_words[wid]->weight;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
WordId TemplatedVocabulary<TDescriptor, F>::transform
    (const TDescriptor& feature) const
{
    if(empty())
    {
        return 0;
    }

    WordId wid;
    transform(feature, wid);
    return wid;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(
  const std::vector<TDescriptor>& features, BowVector &v) const
{
    v.clear();

    if(empty())
    {
        return;
    }

    // normalize
	typename vector<TDescriptor>::const_iterator fit;

    for(fit = features.begin(); fit < features.end(); ++fit)
    {
        WordId id;
        WordValue w;
        // w is the idf value if TF_IDF, 1 if TF

        transform(*fit, id, w);

        // not stopped
        if(w > 0) v.addWeight(id, w);
    }

    v.normalize(L1);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(
    const std::vector<TDescriptor>& features,
    BowVector &v, FeatureVector &fv, int levelsup) const
{
    v.clear();
    fv.clear();

    if(empty()) // safe for subclasses
    {
        return;
    }

    // normalize
    typename vector<TDescriptor>::const_iterator fit;

    unsigned int i_feature = 0;
    for(fit = features.begin(); fit < features.end(); ++fit, ++i_feature)
    {
        WordId id;
        NodeId nid;
        WordValue w;
        // w is the idf value

        transform(*fit, id, w, &nid, levelsup);

        if(w > 0) // not stopped
        {
            v.addWeight(id, w);
            fv.addFeature(nid, i_feature);
        }

        // add yolo result as feature
        cv::Mat desc = *fit;
        int yolo_index = desc.at<unsigned char>(0, 32);
        //int yolo_index = fit->at<uint8_t>(0, 32);
        if (yolo_index != 0 && yolo_index != 1)
        {
            NodeId yoloid = yolo_index + m_words.size();
            v.addWeight(yoloid, 0.01);
            fv.addFeature(yoloid, i_feature);
        }
    }

    v.normalize(L1);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
inline double TemplatedVocabulary<TDescriptor,F>::score
    (const BowVector &v1, const BowVector &v2) const
{
    return m_scoring_object->score(v1, v2);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform
    (const TDescriptor &feature, WordId &id) const
{
    WordValue weight;
    transform(feature, id, weight);
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::transform(const TDescriptor &feature,
    WordId &word_id, WordValue &weight, NodeId *nid, int levelsup) const
{
    // propagate the feature down the tree
    vector<NodeId> nodes;
    typename vector<NodeId>::const_iterator nit;

    // level at which the node must be stored in nid, if given
    const int nid_level = m_L - levelsup;
    if(nid_level <= 0 && nid != NULL) *nid = 0; // root

    NodeId final_id = 0; // root
    int current_level = 0;

    do
    {
        ++current_level;
        nodes = m_nodes[final_id].children;
        final_id = nodes[0];

        double best_d = F::distance(feature, m_nodes[final_id].descriptor);

        for(nit = nodes.begin() + 1; nit != nodes.end(); ++nit)
        {
            NodeId id = *nit;
            double d = F::distance(feature, m_nodes[id].descriptor);
            if(d < best_d)
            {
                best_d = d;
                final_id = id;
            }
        }

        if(nid != NULL && current_level == nid_level)
            *nid = final_id;

    } while( !m_nodes[final_id].isLeaf() );

    // turn node id into word id
    word_id = m_nodes[final_id].word_id;
    weight = m_nodes[final_id].weight;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
NodeId TemplatedVocabulary<TDescriptor,F>::getParentNode
    (WordId wid, int levelsup) const
{
    NodeId ret = m_words[wid]->id; // node id
    while(levelsup > 0 && ret != 0) // ret == 0 --> root
    {
        --levelsup;
        ret = m_nodes[ret].parent;
    }
    return ret;
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
void TemplatedVocabulary<TDescriptor,F>::getWordsFromNode
    (NodeId nid, std::vector<WordId> &words) const
{
    words.clear();

    if(m_nodes[nid].isLeaf())
    {
        words.push_back(m_nodes[nid].word_id);
    }
    else
    {
        words.reserve(m_k); // ^1, ^2, ...

        vector<NodeId> parents;
        parents.push_back(nid);

        while(!parents.empty())
        {
            NodeId parentid = parents.back();
            parents.pop_back();

            const vector<NodeId> &child_ids = m_nodes[parentid].children;
            vector<NodeId>::const_iterator cit;

            for(cit = child_ids.begin(); cit != child_ids.end(); ++cit)
            {
                const Node &child_node = m_nodes[*cit];

                if(child_node.isLeaf())
                    words.push_back(child_node.word_id);
                else
                    parents.push_back(*cit);

            } // for each child
        } // while !parents.empty
    }
}

// --------------------------------------------------------------------------

template<class TDescriptor, class F>
int TemplatedVocabulary<TDescriptor,F>::stopWords(double minWeight)
{
    int c = 0;
    typename vector<Node*>::iterator wit;
    for(wit = m_words.begin(); wit != m_words.end(); ++wit)
    {
        if((*wit)->weight < minWeight)
        {
            ++c;
            (*wit)->weight = 0;
        }
    }
    return c;
}

// --------------------------------------------------------------------------
template<class TDescriptor, class F>
bool TemplatedVocabulary<TDescriptor,F>::loadFromBinaryFile(const std::string &filename) {
    fstream f;
    f.open(filename.c_str(), ios_base::in|ios::binary);
    unsigned int nb_nodes, size_node;
    f.read((char*)&nb_nodes, sizeof(nb_nodes));
    f.read((char*)&size_node, sizeof(size_node));
    f.read((char*)&m_k, sizeof(m_k));
    f.read((char*)&m_L, sizeof(m_L));
    f.read((char*)&m_scoring, sizeof(m_scoring));
    f.read((char*)&m_weighting, sizeof(m_weighting));
    createScoringObject();

    m_words.clear();
    m_words.reserve(pow((double)m_k, (double)m_L + 1));
    m_nodes.clear();
    m_nodes.resize(nb_nodes+1);
    m_nodes[0].id = 0;
    char* buf = new char [size_node];
    int nid = 1;
    while (!f.eof()) {
        f.read(buf, size_node);
        m_nodes[nid].id = nid;
        // FIXME
        const int* ptr=(int*)buf;
        m_nodes[nid].parent = *ptr;
        //m_nodes[nid].parent = *(const int*)buf;
        m_nodes[m_nodes[nid].parent].children.push_back(nid);
        m_nodes[nid].descriptor = cv::Mat(1, F::L, CV_8U); //F::L
        memcpy(m_nodes[nid].descriptor.data, buf+4, F::L); //F::L
        m_nodes[nid].weight = *(float*)(buf+4+F::L); // F::L
        if (buf[8+F::L]) { // is leaf //F::L
            int wid = m_words.size();
            m_words.resize(wid+1);
            m_nodes[nid].word_id = wid;
            m_words[wid] = &m_nodes[nid];
        }
        else
            m_nodes[nid].children.reserve(m_k);
            nid+=1;
    }
    f.close();

    delete[] buf;
    return true;
}

} // namespace DBoW2

#endif
