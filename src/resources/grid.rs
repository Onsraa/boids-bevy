use bevy::prelude::*;
use crate::globals::*;

#[derive(Resource)]
pub struct GridSettings {
    pub width: f32,
    pub height: f32,
}

impl Default for GridSettings {
    fn default() -> Self {
        Self {
            width: GRID_WIDTH,
            height: GRID_HEIGHT,
        }
    }
}