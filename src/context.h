#ifndef STEREO_CONTEXT_H
#define STEREO_CONTEXT_H

class Viewer;
struct Context {
    std::shared_ptr<Frame> frame_previous_{nullptr};
    std::shared_ptr<Frame> frame_current_{nullptr};
    std::shared_ptr<Viewer> viewer_{nullptr};
};

#endif //STEREO_CONTEXT_H
