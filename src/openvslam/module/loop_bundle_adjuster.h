#ifndef OPENVSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H
#define OPENVSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H

namespace openvslam {

class mapping_module;

namespace data {
class map_database;
} // namespace data

namespace module {

class loop_bundle_adjuster {
public:
    /**
     * Constructor
     */
    explicit loop_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter = 10);

    /**
     * Destructor
     */
    ~loop_bundle_adjuster() = default;

    /**
     * Set the mapping module
     */
    void set_mapping_module(mapping_module* mapper);

    /**
     * Count the number of loop BA execution
     */
    void count_loop_BA_execution();

    /**
     * Abort loop BA externally
     */
    void abort();

    /**
     * Loop BA is running or not
     */
    bool is_running() const;

    /**
     * Run loop BA
     */
    void optimize(const unsigned int identifier);

private:
    //! map database
    data::map_database* map_db_ = nullptr;

    //! mapping module
    mapping_module* mapper_ = nullptr;

    //! number of iteration for optimization
    const unsigned int num_iter_ = 10;

    //-----------------------------------------
    // thread management

    //! mutex for access to pause procedure
    mutable std::mutex mtx_thread_;

    //! number of times loop BA is performed
    unsigned int num_exec_loop_BA_ = 0;

    //! flag to abort loop BA
    bool abort_loop_BA_ = false;

    //! flag which indicates loop BA is running or not
    bool loop_BA_is_running_ = false;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_LOOP_BUNDLE_ADJUSTER_H
