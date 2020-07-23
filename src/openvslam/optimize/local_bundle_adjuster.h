#ifndef OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
#define OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H

namespace openvslam {

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace optimize {

class local_bundle_adjuster {
public:
    /**
     * Constructor
     * @param map_db
     * @param num_first_iter
     * @param num_second_iter
     */
    explicit local_bundle_adjuster(const unsigned int num_first_iter = 5,
                                   const unsigned int num_second_iter = 10);

    /**
     * Destructor
     */
    virtual ~local_bundle_adjuster() = default;

    /**
     * Perform optimization
     * @param curr_keyfrm
     * @param force_stop_flag
     */
    void optimize(const std::shared_ptr<data::keyframe>& curr_keyfrm, bool* const force_stop_flag) const;

private:
    //! number of iterations of first optimization
    const unsigned int num_first_iter_;
    //! number of iterations of second optimization
    const unsigned int num_second_iter_;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_H
