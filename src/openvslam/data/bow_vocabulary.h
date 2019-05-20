#ifndef OPENVSLAM_DATA_BOW_VOCABULARY_H
#define OPENVSLAM_DATA_BOW_VOCABULARY_H

#ifdef USE_DBOW2
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#else
#include <fbow/fbow.h>
#endif // USE_DBOW2

namespace openvslam {
namespace data {

#ifdef USE_DBOW2

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> bow_vocabulary;

#else

typedef fbow::Vocabulary bow_vocabulary;

#endif // USE_DBOW2

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_BOW_VOCABULARY_H
