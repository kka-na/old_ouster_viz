#include <Eigen/Eigen>
#include <atomic>
#include <cassert>
#include <cmath>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <vector>

/*
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2); 
*/

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkImageActor.h>
#include <vtkImageChangeInformation.h>
#include <vtkImageData.h>
#include <vtkImageMapToColors.h>
#include <vtkImageMapper.h>
#include <vtkImageMapper3D.h>
#include <vtkImageProperty.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

#include "colormaps.h"
#include "ouster/autoexposure.h"
#include "ouster/beam_uniformity.h"
#include "ouster/lidar_scan.h"
#include "ouster/os1_util.h"
#include "ouster/viz.h"

namespace ouster {
namespace viz {

using Points = Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

/**
 * Helper function to create vtk lookup tables for the specific color palettes
 **/
vtkSmartPointer<vtkLookupTable> palette_gen(const float c[palette_n][3]) {
    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(palette_n);
    lut->Build();
    for (int i = 0; i < palette_n; i++) {
        lut->SetTableValue(i, c[i][0], c[i][1], c[i][2]);
    }
    return lut;
}

const std::vector<std::pair<std::string, vtkSmartPointer<vtkLookupTable>>>
    palettes = {
        {"Parula", palette_gen(parula)}, {"Viridis", palette_gen(viridis)},
        {"Magma", palette_gen(magma)},   {"Rainbow", palette_gen(rainbow)},
        {"Autumn", palette_gen(autumn)}, {"Grey", palette_gen(grey)}};


/**
 * Specify what quantity to color in the point cloud visualization
 **/
enum ColorMode {
    COLOR_Z,
    COLOR_INTENSITY,
    COLOR_ZINTENSITY,
    COLOR_RINTENSITY,
    COLOR_RANGE
};

const std::vector<std::pair<std::string, ColorMode>> color_modes = {
    {"Z", COLOR_Z},
    {"INTENSITY", COLOR_INTENSITY},
    {"Z+INTENSITY", COLOR_ZINTENSITY},
    {"INTENSITY_TIMES_RANGE", COLOR_RINTENSITY},
    {"RANGE", COLOR_RANGE}};

/**
 * Visualizer options set through user controls
 **/
struct VisualizerConfig {
    int c_palette;     // cloud color palette
    int point_size;    // cloud point size
    int color_mode;    // cloud coloring mode
    bool cycle_range;  // cycle range palette
    bool parallel;     // use parallel projection

    int palette;       // image color palette
    bool image_noise;  // display noise image
    double range_scale;

    int fraction_3d;  // percent of window displaying cloud
};

/**
 * Mutable state for visualizer
 */
struct VisualizerState {
    AutoExposure color_intensity;
    AutoExposure color_zintensity;
    AutoExposure color_rintensity;
    AutoExposure color_noise;
    BeamUniformityCorrector buc;
};

struct LidarScanBuffer {
    std::mutex ls_mtx;
    bool ls_dirty = true;  // false if the 'back' scan is new
    std::unique_ptr<ouster::LidarScan> back;
    std::unique_ptr<ouster::LidarScan> front;
};

/**
 * struct containing everything that is passed from init_viz to run_viz
 **/
struct VizHandle {
    VisualizerConfig config;
    VisualizerState state;
    LidarScanBuffer lsb;
    int W;
    int H;
    std::atomic_bool exit;
};

/**
 * Update data being displayed
 **/
void update(viz::VizHandle& vh, std::unique_ptr<ouster::LidarScan>& ls) {
    assert(ls->W * ls->H == vh.W * vh.H);

    std::unique_lock<std::mutex> ls_guard(vh.lsb.ls_mtx);
    ls.swap(vh.lsb.back);
    vh.lsb.ls_dirty = false;
    ls_guard.unlock();
}

/**
 * Applies filter for scaling the intensity scaling factor based on their range
 **/
void color_range(Eigen::Ref<Eigen::ArrayXd> range,
                 const VisualizerConfig& config) {
    range *= config.range_scale;
    if (config.cycle_range) {
        // range = 0.005 * range + 0.5 * (1.0 - (0.015 * M_PI * range).cos());
        range = 0.5 * (1.0 - (0.018 * M_PI * range).cos());
    } else {
        range = (range * 0.02).min(1.0).max(0.0);
    }
}

/**
 * Generates a point cloud from a lidar scan, by multiplying each pixel in the
 * lidar scan by a vector pointing radially outward
 **/
void lidar_scan_to_point_cloud(ouster::LidarScan& ls, Points& xyz) {
    assert(xyz.rows() == ls.W * ls.H);
    assert(xyz.cols() == 3);

    xyz.col(0) = ls.x();
    xyz.col(1) = ls.y();
    xyz.col(2) = ls.z();
}

/**
 * Update scalars used to color points
 **/
void update_color_key(const VisualizerConfig& config, VisualizerState& state,
                      const Points& xyz, Eigen::Ref<Eigen::ArrayXd> intensity,
                      Eigen::Ref<Eigen::ArrayXd> range,
                      std::vector<double>& color_key) {
    const int n = xyz.rows();
    assert(intensity.size() == n);
    assert(range.size() == n);
    assert(color_key.size() == (size_t)n);

    Eigen::Map<Eigen::ArrayXd> key_eigen(color_key.data(), n);

    switch (color_modes[config.color_mode].second) {
        case COLOR_Z:
            key_eigen = ((1.5 + xyz.col(2)) * 0.1).abs().sqrt();
            break;
        case COLOR_INTENSITY:
            key_eigen = Eigen::Map<const Eigen::ArrayXd>(intensity.data(), n);
            state.color_intensity(key_eigen);
            break;
        case COLOR_ZINTENSITY:
            key_eigen = Eigen::Map<const Eigen::ArrayXd>(intensity.data(), n);
            key_eigen += ((1.5 + xyz.col(2)) * 0.05).abs().sqrt();
            state.color_zintensity(key_eigen);
            break;
        case COLOR_RINTENSITY:
            key_eigen =
                (Eigen::Map<const Eigen::ArrayXd>(range.data(), n) + 3.0) *
                Eigen::Map<const Eigen::ArrayXd>(intensity.data(), n);
            state.color_rintensity(key_eigen);
            break;
        case COLOR_RANGE:
            key_eigen = Eigen::Map<const Eigen::ArrayXd>(range.data(), n);
            color_range(key_eigen, config);
            break;
        default:
            cerr << "Invalid render mode" << endl;
    }
}

/**
 * Inserts lidar scan into frame so that it can be rendered
 **/
void update_images(
    ouster::LidarScan& ls,
    Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& arr,
    const std::vector<int>& px_offset, const VisualizerConfig& config,
    VisualizerState& state) {
    using MapXXd = Eigen::Map<Eigen::ArrayXXd>;
    using MapXXdr = Eigen::Map<
        Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

    MapXXd r{ls.range().data(), ls.H, ls.W};
    MapXXd i{ls.intensity().data(), ls.H, ls.W};
    MapXXd n{ls.noise().data(), ls.H, ls.W};

    MapXXdr dst{arr.data(), 3 * ls.H, ls.W};

    // de-stagger and covert to row-major
    for (int u = 0; u < ls.H; u++) {
        const int ofs = px_offset[u];
        dst.row(1 * ls.H - u - 1) << r.row(u).tail(ls.W - ofs),
            r.row(u).head(ofs);
        dst.row(2 * ls.H - u - 1) << i.row(u).tail(ls.W - ofs),
            i.row(u).head(ofs);
        dst.row(3 * ls.H - u - 1) << n.row(u).tail(ls.W - ofs),
            n.row(u).head(ofs);
    }

    Eigen::ArrayXXd noise_image = dst.bottomRows(ls.H);
    state.buc.correct(noise_image);

    const int N = ls.W * ls.H;
    color_range(Eigen::Map<Eigen::ArrayXd>{arr.data(), N}, config);
    state.color_intensity(Eigen::Map<Eigen::ArrayXd>{arr.data() + N, N});
    if (config.image_noise) {
        state.color_noise(Eigen::Map<Eigen::ArrayXd>{noise_image.data(), N});
        dst.bottomRows(ls.H) = noise_image;
    }
};


class MakeStyle : public vtkInteractorStyleTrackballCamera {
   public:
    static MakeStyle* New();
    vtkTypeMacro(MakeStyle, vtkInteractorStyleTrackballCamera);

    VizHandle* vh;
    vtkSmartPointer<vtkCamera> camera;
    vtkSmartPointer<vtkRenderer> renderer;
    std::function<void()> config_updated = [] {};
};
vtkStandardNewMacro(MakeStyle);

vtkSmartPointer<vtkActor> init_cloud_actor(
    vtkSmartPointer<vtkPoints>& points,
    vtkSmartPointer<vtkDoubleArray>& color) {
    auto vtk_cloud = vtkSmartPointer<vtkPolyData>::New();
    vtk_cloud->SetPoints(points);
    vtk_cloud->GetPointData()->SetScalars(color);
    vtk_cloud->GetPointData()->SetActiveScalars("DepthArray");

    auto vfilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vfilter->AddInputData(vtk_cloud);
    vfilter->Update();

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(vfilter->GetOutputPort());
    mapper->SetColorModeToDefault();
    mapper->SetScalarRange(0.0, 1.0);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    return actor;
}

/**
 * Run visualizer rendering loop
 **/
void run_viz(VizHandle& vh) {
    const size_t n_points = vh.W * vh.H;

    //Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> image_data{3 * vh.H, vh.W};

    Points pc_render{n_points, 3};
    std::vector<double> color_key(n_points, 0.0);

    std::vector<int> px_offset = OS1::get_px_offset(vh.W);

    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    vtkDoubleArray::SafeDownCast(points->GetData())
        ->SetArray(pc_render.data(), n_points * 3, 1);

    auto color = vtkSmartPointer<vtkDoubleArray>::New();
    color->SetName("DepthArray");
    color->SetArray(color_key.data(), n_points, 1);

    auto cloud_actor = init_cloud_actor(points, color);

    cloud_actor->GetProperty()->SetPointSize(2);
    cloud_actor->GetMapper()->SetLookupTable(palette_gen(viridis));

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0, 0, 0);
    renderer->AddActor(cloud_actor);

    auto camera = renderer->GetActiveCamera();
    camera->SetPosition(-5, 0, 8);
    camera->SetViewUp(1, 0, 0);

    auto render_window = vtkSmartPointer<vtkRenderWindow>::New();
    render_window->SetSize(1280  , 720);
    render_window->AddRenderer(renderer);

    // vtk callback that calls a std::function in client data
    auto fn_cb = [](vtkObject*, long unsigned int, void* clientData, void*) {
        (*static_cast<std::function<void()>*>(clientData))();
    };


    // render callback
    std::function<void()> on_render = [&]() {
        // check if we're exiting
        //if (vh.exit) render_window_interactor->ExitCallback();

        // swap in new frame
        {
            std::unique_lock<std::mutex> ls_guard(vh.lsb.ls_mtx);
            // already rendered this data
            if (vh.lsb.ls_dirty) return;
            vh.lsb.front.swap(vh.lsb.back);
            vh.lsb.ls_dirty = true;
        }

        // update data backing visualization: pc_render, color_key, image_data
        lidar_scan_to_point_cloud(*vh.lsb.front, pc_render);
        vh.config.c_palette = 3;
        update_color_key(vh.config, vh.state, pc_render,vh.lsb.front->intensity(), vh.lsb.front->range(), color_key);

        points->Modified();
        color->Modified();
        //std::cout<<"on_render"<<std::endl;
        render_window->Render();
    };

    auto timer_cb = vtkSmartPointer<vtkCallbackCommand>::New();
    timer_cb->SetCallback(fn_cb);
    timer_cb->SetClientData(&on_render);


    //auto style = vtkSmartPointer<KeyPressInteractorStyle>::New();
    auto style = vtkSmartPointer<MakeStyle>::New();
    style->vh = &vh;
    style->camera = camera;
    style->renderer = renderer;
    style->SetCurrentRenderer(renderer);
    style->config_updated = [&]() {
        //image_color->SetLookupTable(palettes[vh.config.palette].second);

        cloud_actor->GetProperty()->SetPointSize(5);
        cloud_actor->GetMapper()->SetLookupTable(palette_gen(magma));

        //renderer_2d->SetViewport(0, vh.config.fraction_3d / 100.0, 1, 1);
        //renderer->SetViewport(0, 0, 1, vh.config.fraction_3d / 100.0);
        renderer->SetViewport(0,0, 1, 1);

        //fill_viewport(renderer_2d, image_change->GetOutput(), vh.config);

        if (camera->GetParallelProjection() != vh.config.parallel) {
            camera->SetParallelProjection(vh.config.parallel);
            renderer->ResetCamera();
            renderer->ResetCameraClippingRange();
            camera->SetFocalPoint(0, 0, 0);
        }

        render_window->Render();
    };

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor;
    render_window_interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    render_window_interactor->SetRenderWindow(render_window);
    render_window_interactor->SetInteractorStyle(style);
    render_window_interactor->Initialize();
    render_window_interactor->AddObserver(vtkCommand::TimerEvent, timer_cb);
    render_window_interactor->CreateRepeatingTimer(16);

    //style->config_updated();

    render_window_interactor->Start();
  


}

/**
 * Exit visualizer
 **/
void shutdown(VizHandle& vh) { vh.exit = true; }

/**
 * Initializes the visualizer based on user-given parameters
 **/
std::shared_ptr<VizHandle> init_viz(int W, int H) {
    auto vh = std::make_shared<VizHandle>();

    vh->config.fraction_3d = 70;

    vh->H = H;
    vh->W = W;
    vh->exit = false;

    vh->config.palette = 1; 
    vh->config.c_palette = 1;
    vh->config.point_size = 2;

    vh->config.color_mode = 2;
    vh->config.cycle_range = false;
    vh->config.parallel = false;
    vh->config.image_noise = true;
    vh->config.range_scale = 0.005;

    vh->lsb.back =
        std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));

    vh->lsb.front =
        std::unique_ptr<ouster::LidarScan>(new ouster::LidarScan(W, H));

    return vh;
}
}  // namespace viz
}  // namespace ouster
