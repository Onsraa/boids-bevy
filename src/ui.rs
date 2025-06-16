use bevy::prelude::*;
use bevy_egui::*;
use crate::BoidSettings;
use crate::resources::grid::GridSettings;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: true,
        });
        app.add_systems(EguiContextPass, ui_system);
    }
}

fn ui_system(mut contexts: EguiContexts, mut boid_settings: ResMut<BoidSettings>, mut grid_settings: ResMut<GridSettings>) {
    egui::Window::new("Parameters").show(contexts.ctx_mut(), |ui| {
        ui.heading("Boids Settings");
        ui.label("Radius");
        ui.add(egui::Slider::new(&mut boid_settings.separation_radius, 0.0..=1000.0).text("Separation Radius"));
        ui.add(egui::Slider::new(&mut boid_settings.alignment_radius, 0.0..=1000.0).text("Alignment Radius"));
        ui.add(egui::Slider::new(&mut boid_settings.cohesion_radius, 0.0..=1000.0).text("Cohesion Radius"));
        ui.label("Weight");
        ui.add(egui::Slider::new(&mut boid_settings.separation_weight, 0.0..=100.0).text("Separation Weight"));
        ui.add(egui::Slider::new(&mut boid_settings.alignment_weight, 0.0..=100.0).text("Alignment Weight"));
        ui.add(egui::Slider::new(&mut boid_settings.cohesion_weight, 0.0..=100.0).text("Cohesion Weight"));

        ui.heading("Grid Settings");
        ui.add(egui::Slider::new(&mut grid_settings.width, 0.0..=1000.0).text("Width"));
        ui.add(egui::Slider::new(&mut grid_settings.height, 0.0..=1000.0).text("Height"));
    });
}
